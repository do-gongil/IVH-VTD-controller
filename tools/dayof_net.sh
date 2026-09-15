#!/bin/bash
# 대회 당일: 운영측 VTD PC 와 이더넷 직결용 고정 IP 설정 + 연결 점검
#   sudo bash tools/dayof_net.sh <우리_IP/prefix> <VTD_PC_IP>   예) 192.168.10.20/24 192.168.10.10
set -euo pipefail
IF=enp4s0
OUR=${1:?우리 IP/prefix (예 192.168.10.20/24)}
VTD=${2:?VTD PC IP}

echo "== 링크 상태 =="; ip link show "$IF" | head -2
nmcli -t -f NAME con show | grep -qx hlfma && nmcli con delete hlfma >/dev/null
nmcli con add type ethernet ifname "$IF" con-name hlfma ip4 "$OUR" ipv4.method manual autoconnect yes >/dev/null
nmcli con up hlfma >/dev/null
echo "== IP =="; ip -4 addr show "$IF" | grep inet
echo "== ping $VTD =="; ping -c 3 -W 1 "$VTD" || { echo "!! ping 실패: 케이블/IP 확인"; exit 1; }
echo "== TCP 9910 =="; timeout 3 bash -c "cat < /dev/null > /dev/tcp/$VTD/9910" && echo "9910 열림 (시뮬레이션 실행 중)" || echo "9910 닫힘 (시뮬레이션 시작 전이면 정상 — 제어기는 재시도함)"
