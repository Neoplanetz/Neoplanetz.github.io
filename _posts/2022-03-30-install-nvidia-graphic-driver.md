# Nvidia Graphic Driver 설치법

## 1. Nvidia 그래픽 카드 확인
- 본인 PC에 장착된 Nvidia 그래픽 카드 확인
  ```bash
  # 장착된 장치 확인
  $ ubuntu-drivers devices
  == /sys/devices/pci0000:00/0000:00:01.0/0000:01:00.0 ==
  modalias : pci:v000010DEd00001B80sv0000103Csd000082FBbc03sc00i00
  vendor   : NVIDIA Corporation
  model    : GP104 [GeForce GTX 1080]
  driver   : nvidia-driver-418-server - distro non-free
  driver   : nvidia-driver-390 - distro non-free
  driver   : nvidia-driver-510 - distro non-free
  driver   : nvidia-driver-470-server - distro non-free
  driver   : nvidia-driver-470 - distro non-free recommended
  driver   : nvidia-driver-450-server - distro non-free
  driver   : nvidia-driver-510-server - distro non-free
  driver   : xserver-xorg-video-nouveau - distro free builtin
  ```
  > 현재 장착된 그래픽카드는 GTX 1080이고 추천하는 드라이버는 nvidia-driver-470 버전
  

## 2. 해당되는 추천 Nvidia 그래픽 드라이버 설치
- 본인 PC에 장착된 그래픽 카드에 맞는 드라이버 설치 (2 방법 중 1개 선택해 설치)
  ```bash
  # 1. 해당 장치에 맞는 드라이버 자동 설치
  $ sudo ubuntu-drivers autoinstall

  # 2. 수동 설치
  $ sudo apt install nvidia-driver-470 -y

  ```

## 3. Nvidia 그래픽 카드 설치 완료
- 그래픽 드라이버 설치 후 재부팅 하면 설정 완료
  ```bash
  $ sudo reboot
  ```