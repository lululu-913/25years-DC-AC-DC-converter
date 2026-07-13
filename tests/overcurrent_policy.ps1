$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$sourcePath = Join-Path $repoRoot "main.c"
$source = Get-Content -Raw $sourcePath

function Assert-Matches {
    param(
        [string]$Text,
        [string]$Pattern,
        [string]$Message
    )
    if ($Text -notmatch $Pattern) {
        throw $Message
    }
}

function Assert-NotMatches {
    param(
        [string]$Text,
        [string]$Pattern,
        [string]$Message
    )
    if ($Text -match $Pattern) {
        throw $Message
    }
}

$overcurrentBlockMatch = [regex]::Match(
    $source,
    "(?s)// ===== Step 3:.*?// ===== Step 4:"
)

if (-not $overcurrentBlockMatch.Success) {
    throw "Could not find the overcurrent protection block in main.c"
}

$overcurrentBlock = $overcurrentBlockMatch.Value

Assert-NotMatches `
    -Text $overcurrentBlock `
    -Pattern "tag\s*=\s*0\s*;" `
    -Message "Overcurrent protection must not force tag=0, because that stops the inverter."

Assert-Matches `
    -Text $source `
    -Pattern "#define\s+OVERCURRENT_PROTECTION_ENABLED\s+0" `
    -Message "Software overcurrent protection must be temporarily disabled by macro."

Assert-Matches `
    -Text $source `
    -Pattern "#define\s+OVERCURRENT_CONFIRM_COUNT\s+5" `
    -Message "Overcurrent protection must require 5 consecutive samples over the limit."

Assert-Matches `
    -Text $overcurrentBlock `
    -Pattern "(?s)#if\s+OVERCURRENT_PROTECTION_ENABLED.*overcurrent_count\s*\+\+.*#endif" `
    -Message "Overcurrent counting code must be retained but compiled out while protection is disabled."

Assert-Matches `
    -Text $overcurrentBlock `
    -Pattern "(?s)#if\s+OVERCURRENT_PROTECTION_ENABLED.*overcurrent_count\s*>=\s*OVERCURRENT_CONFIRM_COUNT.*#endif" `
    -Message "Overcurrent trip code must be retained but compiled out while protection is disabled."

Assert-Matches `
    -Text $overcurrentBlock `
    -Pattern "(?s)#if\s+OVERCURRENT_PROTECTION_ENABLED.*tag\s*=\s*1\s*;.*#endif" `
    -Message "Confirmed overcurrent fallback must be retained but compiled out while protection is disabled."

Assert-Matches `
    -Text $overcurrentBlock `
    -Pattern "(?s)#if\s+OVERCURRENT_PROTECTION_ENABLED.*if\s*\(\s*\(tag\s*==\s*2\)\s*&&\s*\(overcurrent_count\s*>=\s*OVERCURRENT_CONFIRM_COUNT\)\s*\).*#endif" `
    -Message "Confirmed overcurrent fallback must remain limited to tag=2 feedback mode when re-enabled."

Assert-Matches `
    -Text $overcurrentBlock `
    -Pattern "(?s)#if\s+OVERCURRENT_PROTECTION_ENABLED.*rect_overcurrent_latch\s*=\s*1\s*;.*#endif" `
    -Message "Rectifier shutdown latch must be retained but compiled out while protection is disabled."

Assert-Matches `
    -Text $overcurrentBlock `
    -Pattern "(?s)#if\s+OVERCURRENT_PROTECTION_ENABLED.*Rectifier_ForceOff\s*\(\s*\)\s*;.*#endif" `
    -Message "Rectifier force-off on overcurrent must be retained but compiled out while protection is disabled."

Assert-Matches `
    -Text $source `
    -Pattern "if\s*\(\s*\(tag\s*==\s*2\)\s*&&\s*\(rect_overcurrent_latch\s*==\s*0\)\s*\)" `
    -Message "Rectifier PWM updates in tag=2 must be blocked while rect_overcurrent_latch is set."

Write-Host "Overcurrent policy check passed."
