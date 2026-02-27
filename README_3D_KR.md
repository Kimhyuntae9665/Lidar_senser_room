# LiDAR 실시간 2D 시각화

## 1) 설치
```bash
cd /home/astro/lidar_test
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements_3d.txt
```

## 2) 장치 권한(필요 시)
`/dev/ttyUSB0` 접근 권한이 없으면:
```bash
sudo usermod -aG dialout $USER
# 로그아웃/로그인 후 적용
```

## 3) 실행
```bash
cd /home/astro/lidar_test
source .venv/bin/activate
python3 lidar_3d_viewer.py --port /dev/ttyUSB0 --history-scans 60 --draw-every 2
```

입력 버퍼 여유를 늘려 통신 깨짐을 줄이고 싶으면:
```bash
python3 lidar_3d_viewer.py --port /dev/ttyUSB0 --max-buf-meas 20000 --history-scans 60
```

## 4) 포인트 저장
종료 시 최근 2D 프레임을 PLY로 저장(z=0):
```bash
python3 lidar_3d_viewer.py --port /dev/ttyUSB0 --save-ply out.ply
```

## 참고
단일 2D LiDAR(A1)는 기본적으로 한 평면만 측정합니다.

## 트러블슈팅
- `RPLidarException: Check bit not equal to 1`:
  - USB 전원/케이블/허브 노이즈 또는 직렬 프레임 깨짐일 때 자주 발생합니다.
  - 현재 뷰어는 해당 오류를 소프트 복구(스트림 재동기화)로 처리합니다.
  - A1은 보통 `115200`이 맞습니다.
```bash
python3 lidar_3d_viewer.py --port /dev/ttyUSB0 --baudrate 115200
```
