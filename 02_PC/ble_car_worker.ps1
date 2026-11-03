param(
    [Parameter(Mandatory=$true)][string]$Address,
    [switch]$Inspect
)
# Windows PowerShell 5.1 WinRT bridge. No Python BLE packages are required.
# Connecting/inspecting never sends a car command. Writes require stdin JSON.
$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = New-Object System.Text.UTF8Encoding($false)
Add-Type -AssemblyName System.Runtime.WindowsRuntime
[void][Windows.Devices.Bluetooth.BluetoothLEDevice,Windows.Devices.Bluetooth,ContentType=WindowsRuntime]
[void][Windows.Devices.Bluetooth.GenericAttributeProfile.GattDeviceServicesResult,Windows.Devices.Bluetooth,ContentType=WindowsRuntime]
[void][Windows.Devices.Bluetooth.GenericAttributeProfile.GattCharacteristicsResult,Windows.Devices.Bluetooth,ContentType=WindowsRuntime]
[void][Windows.Devices.Bluetooth.GenericAttributeProfile.GattCommunicationStatus,Windows.Devices.Bluetooth,ContentType=WindowsRuntime]
[void][Windows.Devices.Bluetooth.GenericAttributeProfile.GattCharacteristicProperties,Windows.Devices.Bluetooth,ContentType=WindowsRuntime]
[void][Windows.Devices.Bluetooth.GenericAttributeProfile.GattWriteOption,Windows.Devices.Bluetooth,ContentType=WindowsRuntime]
[void][Windows.Devices.Bluetooth.GenericAttributeProfile.GattCharacteristic,Windows.Devices.Bluetooth,ContentType=WindowsRuntime]
[void][Windows.Security.Cryptography.CryptographicBuffer,Windows.Security.Cryptography,ContentType=WindowsRuntime]
[void][Windows.Storage.Streams.IBuffer,Windows.Storage.Streams,ContentType=WindowsRuntime]

# CryptographicBuffer returns System.__ComObject. PowerShell 5.1's dynamic
# overload binder fails to match IBuffer even though COM supports the interface.
# Resolve the exact WinRT method once; reflection performs the interface cast.
$gattWriteMethod = [Windows.Devices.Bluetooth.GenericAttributeProfile.GattCharacteristic].GetMethod(
    'WriteValueAsync',
    [Type[]]@([Windows.Storage.Streams.IBuffer], [Windows.Devices.Bluetooth.GenericAttributeProfile.GattWriteOption]))

function Start-GattWrite($Target, $Data, $WriteOption) {
    if ($null -eq $gattWriteMethod) { throw 'The typed GATT write API is unavailable' }
    [object[]]$writeArguments = @($Data, $WriteOption)
    return $gattWriteMethod.Invoke($Target, $writeArguments)
}

$asTaskMethod = [System.WindowsRuntimeSystemExtensions].GetMethods() | Where-Object {
    $_.Name -eq 'AsTask' -and $_.IsGenericMethod -and
    $_.GetGenericArguments().Count -eq 1 -and $_.GetParameters().Count -eq 1 -and
    $_.GetParameters()[0].ParameterType.Name -eq 'IAsyncOperation`1'
} | Select-Object -First 1

function Await-WinRT($Operation, [Type]$ResultType, [int]$TimeoutMs=15000) {
    $task = $asTaskMethod.MakeGenericMethod($ResultType).Invoke($null, @($Operation))
    if (-not $task.Wait($TimeoutMs)) {
        $Operation.Cancel()
        throw 'BLE operation timed out. Keep the module powered and disconnect other clients.'
    }
    return $task.GetAwaiter().GetResult()
}

function Send-Event($Value) {
    [Console]::WriteLine(($Value | ConvertTo-Json -Depth 5 -Compress))
    [Console]::Out.Flush()
}

$device = $null
$service = $null
$characteristic = $null
$motionSent = $false
try {
    $hex = $Address -replace '[:-]', ''
    if ($hex -notmatch '^[0-9a-fA-F]{12}$') { throw 'Invalid BLE address' }
    $number = [Convert]::ToUInt64($hex, 16)
    $device = Await-WinRT ([Windows.Devices.Bluetooth.BluetoothLEDevice]::FromBluetoothAddressAsync($number)) ([Windows.Devices.Bluetooth.BluetoothLEDevice])
    if ($null -eq $device) {
        $device = Await-WinRT ([Windows.Devices.Bluetooth.BluetoothLEDevice]::FromBluetoothAddressAsync($number, [Windows.Devices.Bluetooth.BluetoothAddressType]::Random)) ([Windows.Devices.Bluetooth.BluetoothLEDevice])
    }
    if ($null -eq $device) { throw 'Windows could not open the BLE device. Pair it and keep it powered.' }
    $result = Await-WinRT ($device.GetGattServicesAsync([Windows.Devices.Bluetooth.BluetoothCacheMode]::Uncached)) ([Windows.Devices.Bluetooth.GenericAttributeProfile.GattDeviceServicesResult])
    if ($result.Status.ToString() -ne 'Success') { throw ('GATT service discovery: ' + $result.Status) }
    $services = @($result.Services)
    $service = $services | Where-Object { $_.Uuid -eq [Guid]'0000ffe0-0000-1000-8000-00805f9b34fb' } | Select-Object -First 1
    $serviceIds = @($services | ForEach-Object { $_.Uuid.ToString() })
    foreach ($other in $services) { if ($other -ne $service) { $other.Dispose() } }
    if ($null -eq $service) { throw ('Expected FFE0 service was not found. Discovered: ' + ($serviceIds -join ', ')) }
    $chars = Await-WinRT ($service.GetCharacteristicsAsync([Windows.Devices.Bluetooth.BluetoothCacheMode]::Uncached)) ([Windows.Devices.Bluetooth.GenericAttributeProfile.GattCharacteristicsResult])
    if ($chars.Status.ToString() -ne 'Success') { throw ('GATT characteristic discovery: ' + $chars.Status) }
    $details = @($chars.Characteristics | ForEach-Object { @{uuid=$_.Uuid.ToString(); properties=$_.CharacteristicProperties.ToString()} })
    Send-Event @{kind='services'; name=$device.Name; address=$Address; services=$serviceIds; characteristics=$details}
    $writable = @($chars.Characteristics | Where-Object {
        ($_.CharacteristicProperties -band [Windows.Devices.Bluetooth.GenericAttributeProfile.GattCharacteristicProperties]::Write) -or
        ($_.CharacteristicProperties -band [Windows.Devices.Bluetooth.GenericAttributeProfile.GattCharacteristicProperties]::WriteWithoutResponse)
    })
    $preferred = @($writable | Where-Object { $_.Uuid -eq [Guid]'0000ffe1-0000-1000-8000-00805f9b34fb' })
    if ($preferred.Count -eq 1) { $characteristic = $preferred[0] }
    elseif ($writable.Count -eq 1) { $characteristic = $writable[0] }
    else { throw 'No unique writable UART characteristic was found under FFE0. No commands were sent.' }
    $option = [Windows.Devices.Bluetooth.GenericAttributeProfile.GattWriteOption]::WriteWithoutResponse
    if ($characteristic.CharacteristicProperties -band [Windows.Devices.Bluetooth.GenericAttributeProfile.GattCharacteristicProperties]::Write) {
        $option = [Windows.Devices.Bluetooth.GenericAttributeProfile.GattWriteOption]::WriteWithResponse
    }
    Send-Event @{kind='ready'; name=$device.Name; address=$Address; characteristic=$characteristic.Uuid.ToString(); write_option=$option.ToString(); inspect=[bool]$Inspect; worker_revision='auto_interlock_20261020'}
    if (-not $Inspect) {
        while ($null -ne ($line = [Console]::ReadLine())) {
            $request = $line | ConvertFrom-Json
            if ($request.action -eq 'close') { break }
            if ($request.action -ne 'write' -or $request.command -cnotmatch '^[FBLRS+0-9-]$') {
                throw 'Invalid car command'
            }
            $bytes = [Text.Encoding]::ASCII.GetBytes([string]$request.command)
            $buffer = [Windows.Security.Cryptography.CryptographicBuffer]::CreateFromByteArray($bytes)
            if ($request.command -cmatch '^[FBLR]$') { $motionSent = $true }
            $writeTimeout = 3000
            if ($null -ne $request.timeout_ms) {
                $writeTimeout = [int]$request.timeout_ms
                if ($writeTimeout -lt 100 -or $writeTimeout -gt 3000) { throw 'Invalid write timeout' }
            }
            $status = Await-WinRT (Start-GattWrite $characteristic $buffer $option) ([Windows.Devices.Bluetooth.GenericAttributeProfile.GattCommunicationStatus]) $writeTimeout
            if ($status.ToString() -ne 'Success') { throw ('GATT write: ' + $status) }
            if ($request.command -ceq 'S') { $motionSent = $false }
            Send-Event @{kind='written'; command=$request.command; id=$request.id; status=$status.ToString()}
        }
    }
}
catch {
    Send-Event @{kind='error'; message=$_.Exception.Message}
    exit 1
}
finally {
    # If the GUI closes/crashes after sending motion, try STOP before releasing
    # the connection. This is best effort, not a car-side watchdog guarantee.
    if ($motionSent -and $null -ne $characteristic) {
        try {
            $stop = [Windows.Security.Cryptography.CryptographicBuffer]::CreateFromByteArray([byte[]]@(0x53))
            $status = Await-WinRT (Start-GattWrite $characteristic $stop $option) ([Windows.Devices.Bluetooth.GenericAttributeProfile.GattCommunicationStatus]) 1000
            Send-Event @{kind='shutdown_stop'; status=$status.ToString()}
        } catch { Send-Event @{kind='shutdown_stop_failed'; message=$_.Exception.Message} }
    }
    if ($null -ne $service) { $service.Dispose() }
    if ($null -ne $device) { $device.Dispose() }
}
