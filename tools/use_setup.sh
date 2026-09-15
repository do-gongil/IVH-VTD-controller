#!/bin/bash
# VTD 셋업 전환 — Data/Setups/Current 심볼릭 링크를 바꾼다.
#   tools/use_setup.sh              현재 셋업과 선택지 표시
#   tools/use_setup.sh 01_HL_VTD_fast   테스트용(빠른) 셋업으로 전환
#   tools/use_setup.sh 00_HL_VTD        대회 배포 원본으로 복귀
# VTD 실행 중에는 전환하지 않는다 (다음 vtdStart.sh 부터 적용된다).
set -euo pipefail
S="${VTD_ROOT:-$HOME/Hexagon/VTD.2025.2}/Data/Setups"

if [ $# -eq 0 ]; then
    echo "현재: $(basename "$(readlink -f "$S/Current")")"
    echo "선택지:"
    for d in "$S"/00_HL_VTD "$S"/01_HL_VTD_fast; do
        [ -d "$d" ] && echo "  $(basename "$d")"
    done
    exit 0
fi

NAME="$1"
[ -d "$S/$NAME" ] || { echo "셋업 없음: $S/$NAME"; exit 1; }

if pgrep -x simServer >/dev/null; then
    echo "VTD 가 실행 중이다. 먼저 정지할 것:  \$VTD_ROOT/bin/vtdStop.sh"
    exit 1
fi

ln -sfn "$NAME" "$S/Current"
echo "Current -> $(basename "$(readlink -f "$S/Current")")"
python3 - "$S/Current" <<'PY'
import json, sys
from pathlib import Path
p = Path(sys.argv[1]) / "Config/HLVTD/hl_vtd_config.json"
d = json.loads(p.read_text(encoding="utf-8"))
print("  센서:", d["sensors"])
print("  lidarDestinationIp:", d["streaming"]["lidarDestinationIp"])
PY
grep -ohE 'enableDatabasePagerThread="[^"]*"|lodscale="[^"]*"' "$S/Current"/Config/ImageGenerator/*.xml | sort -u | sed 's/^/  IG: /'
