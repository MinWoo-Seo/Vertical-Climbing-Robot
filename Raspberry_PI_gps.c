1. 전원을 켜고 시스템에 로그인하십시오. 터미널을 열고 다음 명령을 입력하여 GPS 모듈 용 패키지를 설치할 수 있습니다.
sudo apt-get update && sudo apt-get -y install gpsd gpsd-clients python-gps
2. gpsd 서비스를 시작하고 제어하십시오.
사용 : 
sudo systemctl enable gpsd.socket

시작 : 
sudo systemctl start gpsd.socket

다시 시작 : 
sudo systemctl restart gpsd.socket

상태 확인 : 
sudo systemctl status gpsd.socket