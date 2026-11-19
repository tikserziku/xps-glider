# Print one raster page at EXACT physical size (no fit-to-page) on a Windows printer queue. Windows PowerShell 5.1.
# The image is placed in millimetres with its top-left corner on the physical corner of the sheet; the driver's
# unprintable border simply clips (our content keeps >= 8 mm from the edge).
# Usage: powershell -File print_png.ps1 -Png <file.png> -Printer '\\server\QUEUE' -Paper 'A4' -WidthMm 210 -HeightMm 297 [-Landscape] [-JobName x] [-DryRun]
#   -DryRun : only report the paper size, printable area and hard margins the driver gives; nothing is sent.
param([Parameter(Mandatory)][string]$Png, [Parameter(Mandatory)][string]$Printer, [Parameter(Mandatory)][string]$Paper,
      [Parameter(Mandatory)][double]$WidthMm, [Parameter(Mandatory)][double]$HeightMm,
      [switch]$Landscape, [string]$JobName = 'print_png', [switch]$DryRun)
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Drawing
$pd = New-Object System.Drawing.Printing.PrintDocument
$pd.PrinterSettings.PrinterName = $Printer
if (-not $pd.PrinterSettings.IsValid) { throw "printer not valid: $Printer" }
$size = $pd.PrinterSettings.PaperSizes | Where-Object { $_.PaperName -eq $Paper } | Select-Object -First 1
if (-not $size) { throw "paper '$Paper' not offered by $Printer" }
$pd.DefaultPageSettings.PaperSize = $size
$pd.DefaultPageSettings.Landscape = [bool]$Landscape
$pd.DefaultPageSettings.Margins = New-Object System.Drawing.Printing.Margins(0, 0, 0, 0)
$pd.DefaultPageSettings.Color = $false
$pd.OriginAtMargins = $false
$pd.PrinterSettings.Copies = 1
$pd.DocumentName = $JobName
$pd.PrintController = New-Object System.Drawing.Printing.StandardPrintController     # no status dialog
$k = 0.254
$ps = $pd.DefaultPageSettings; $a = $ps.PrintableArea
"printer   : $Printer"
"paper     : {0}  {1:N1} x {2:N1} mm  landscape={3}  source={4}" -f $size.PaperName, ($size.Width * $k), ($size.Height * $k), [bool]$Landscape, $ps.PaperSource.SourceName
"printable : x={0:N1} y={1:N1} w={2:N1} h={3:N1} mm (portrait frame)   hard margins X={4:N1} Y={5:N1} mm" -f ($a.X * $k), ($a.Y * $k), ($a.Width * $k), ($a.Height * $k), ($ps.HardMarginX * $k), ($ps.HardMarginY * $k)
$pw = if ($Landscape) { $size.Height * $k } else { $size.Width * $k }
$ph = if ($Landscape) { $size.Width * $k } else { $size.Height * $k }
"image     : {0:N2} x {1:N2} mm on a sheet {2:N1} x {3:N1} mm  ->  scale 1:1, overhang {4:N2} / {5:N2} mm" -f $WidthMm, $HeightMm, $pw, $ph, ($WidthMm - $pw), ($HeightMm - $ph)
if ([math]::Abs($WidthMm - $pw) -gt 1.5 -or [math]::Abs($HeightMm - $ph) -gt 1.5) { throw 'image size does not match the sheet (more than 1.5 mm) - refusing to print' }
if ($DryRun) { 'DRY RUN - nothing sent'; return }
$img = [System.Drawing.Image]::FromFile((Resolve-Path $Png).Path)
$pages = 0
$handler = [System.Drawing.Printing.PrintPageEventHandler] {
  param($s, $e)
  $g = $e.Graphics
  $g.PageUnit = [System.Drawing.GraphicsUnit]::Millimeter
  $g.InterpolationMode = [System.Drawing.Drawing2D.InterpolationMode]::HighQualityBicubic
  # Graphics origin = corner of the printable area; shift back by the hard margin so (0,0) is the sheet corner.
  $hx = $e.PageSettings.HardMarginX * 0.254; $hy = $e.PageSettings.HardMarginY * 0.254
  if ($e.PageSettings.Landscape) { $t = $hx; $hx = $hy; $hy = $t }
  $g.DrawImage($img, [single](-$hx), [single](-$hy), [single]$WidthMm, [single]$HeightMm)
  $e.HasMorePages = $false
  $script:pages++
}
$pd.add_PrintPage($handler)
$pd.Print()
$img.Dispose()
"SENT: job '$JobName', pages $pages, copies 1"
