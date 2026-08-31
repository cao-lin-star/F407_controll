param(
    [string]$ProjectPath = (Join-Path $PSScriptRoot '..\MDK-ARM\Footbath_Chassis_F407.uvprojx')
)

$ErrorActionPreference = 'Stop'
$project = (Resolve-Path -LiteralPath $ProjectPath).Path
[xml]$xml = Get-Content -LiteralPath $project -Raw
$target = $xml.Project.Targets.Target
$groups = $target.Groups

$target.TargetOption.TargetArmAds.Cads.VariousControls.IncludePath =
    '../App/Business/Inc;../App/Function/Inc;../App/Hardware/Inc;' +
    '../../include;../Core/Inc;' +
    '../Middlewares/Third_Party/FreeRTOS/Source/include;' +
    '../Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F;' +
    '../Drivers/STM32F4xx_HAL_Driver/Inc;' +
    '../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy;' +
    '../Drivers/CMSIS/Device/ST/STM32F4xx/Include;' +
    '../Drivers/CMSIS/Include'

$managedGroups = @(
    'Application/Footbath App',
    'Application/Business Layer',
    'Application/Function Layer',
    'Application/Hardware Layer',
    'Middleware/FreeRTOS'
)
foreach ($group in @($groups.Group | Where-Object { $_.GroupName -in $managedGroups })) {
    [void]$groups.RemoveChild($group)
}

function Add-SourceGroup {
    param(
        [string]$Name,
        [object[]]$Files
    )
    $group = $xml.CreateElement('Group')
    $groupName = $xml.CreateElement('GroupName')
    $groupName.InnerText = $Name
    [void]$group.AppendChild($groupName)
    $filesNode = $xml.CreateElement('Files')

    foreach ($item in $Files) {
        $file = $xml.CreateElement('File')
        $fileName = $xml.CreateElement('FileName')
        $fileName.InnerText = $item[0]
        $fileType = $xml.CreateElement('FileType')
        $fileType.InnerText = '1'
        $filePath = $xml.CreateElement('FilePath')
        $filePath.InnerText = $item[1]
        [void]$file.AppendChild($fileName)
        [void]$file.AppendChild($fileType)
        [void]$file.AppendChild($filePath)
        [void]$filesNode.AppendChild($file)
    }
    [void]$group.AppendChild($filesNode)
    [void]$groups.AppendChild($group)
}

Add-SourceGroup 'Application/Business Layer' @(
    @('chassis_app.c', '../App/Business/Src/chassis_app.c'),
    @('chassis_tasks.c', '../App/Business/Src/chassis_tasks.c')
)
Add-SourceGroup 'Application/Function Layer' @(
    @('ps2_remote.c', '../App/Function/Src/ps2_remote.c'),
    @('sensor_hub.c', '../App/Function/Src/sensor_hub.c'),
    @('debug_service.c', '../App/Function/Src/debug_service.c')
)
Add-SourceGroup 'Application/Hardware Layer' @(
    @('board_f407.c', '../App/Hardware/Src/board_f407.c'),
    @('uart_transport_f407.c', '../App/Hardware/Src/uart_transport_f407.c'),
    @('debug_console.c', '../App/Hardware/Src/debug_console.c')
)
Add-SourceGroup 'Middleware/FreeRTOS' @(
    @('tasks.c', '../Middlewares/Third_Party/FreeRTOS/Source/tasks.c'),
    @('queue.c', '../Middlewares/Third_Party/FreeRTOS/Source/queue.c'),
    @('list.c', '../Middlewares/Third_Party/FreeRTOS/Source/list.c'),
    @('port.c', '../Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F/port.c')
)

$settings = [System.Xml.XmlWriterSettings]::new()
$settings.Indent = $true
$settings.NewLineChars = [Environment]::NewLine
$settings.Encoding = [System.Text.UTF8Encoding]::new($false)
$writer = [System.Xml.XmlWriter]::Create($project, $settings)
try {
    $xml.Save($writer)
}
finally {
    $writer.Close()
}

Write-Host "Keil layer and FreeRTOS groups restored: $project"
