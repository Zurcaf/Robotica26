# Cria Lab2\.venv e instala tudo (Windows / PowerShell).
# Uso, a partir de Lab2\:   powershell -ExecutionPolicy Bypass -File setup.ps1
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

function Run([scriptblock]$cmd) {
    & $cmd
    if ($LASTEXITCODE -ne 0) { throw "Falhou: $cmd (se o erro falar de nome demasiado longo, usa uma pasta com caminho mais curto)" }
}

if (Get-Command py -ErrorAction SilentlyContinue) { Run { py -3.11 -m venv .venv } } else { Run { python -m venv .venv } }
Run { .\.venv\Scripts\python.exe -m pip install --upgrade pip }
Run { .\.venv\Scripts\python.exe -m pip install -r requirements.txt }

Write-Host ""
Write-Host "Pronto. Ativar com:  .venv\Scripts\activate"
Write-Host "Testar com:          python src\kinematics.py"
Write-Host "Notebook:            jupyter lab lab2.ipynb"
