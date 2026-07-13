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

Assert-Matches `
    -Text $source `
    -Pattern "tag_list\s*\[\s*TAG_LIST_LEN\s*\]\s*=\s*\{\s*0\s*,\s*1\s*,\s*2\s*\}" `
    -Message "KEY3 mode cycle must be 0 -> 1 -> 2 -> 0."

Assert-Matches `
    -Text $source `
    -Pattern "#define\s+TAG2_FREQ_HZ\s+50\.0f" `
    -Message "Feedback fixed frequency constants must belong to tag2."

Assert-Matches `
    -Text $source `
    -Pattern "#define\s+TAG2_U_BUS_REF\s+32\.0f" `
    -Message "Feedback voltage reference constants must belong to tag2."

Assert-Matches `
    -Text $source `
    -Pattern "#define\s+TAG2_I_PID_REF\s+2\.0f" `
    -Message "Feedback current reference constants must belong to tag2."

Assert-NotMatches `
    -Text $source `
    -Pattern "TAG3|tag\s*==\s*3|tag\s*=\s*3|tag=3" `
    -Message "tag3 must be removed from mode logic and constants."

Assert-Matches `
    -Text $source `
    -Pattern "if\s*\(\s*tag\s*==\s*2\s*\).*?能量回馈" `
    -Message "KEY_Control must treat tag2 as the feedback mode."

Write-Host "Tag mode policy check passed."
