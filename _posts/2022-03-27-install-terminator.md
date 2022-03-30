# Terminator 설치 및 단축키 문제 해결 및 사용법

## 1. Terminator 설치

```bash
$ sudo apt install terminator -y
```

## 2. 수직으로 창 나누기 단축키 문제 해결하기
- 기본적으로 수직 창 나누기 단축키는 Ctrl + Shift + E 인데, 우분투 20.04 버전에서는 Emoji 단축키가 Ctrl + Shift + E 로 동일해서 터미네이터 수직 창 나누기 단축키가 작동하지 않음
- 이 문제를 해결하기위해 Emoji 단축키를 설정 해제하여 문제를 해결
- 우선, 새롭게 터미널을 열어 아래와 같이 ibus 설정창의 띄움

```bash
$ ibus-setup
```

- 'IBus Preferences'의 'Emoji' 메뉴로 이동하여 '...'버튼을 선택
    
    ![IBus_setup](/assets/img/ibus_setup.png)
    
- 현재 등록되어있는 단축키를 선택 후, Delete 버튼을 눌러 삭제하고 OK 버튼 선택
    
    ![Emoji_key_shortcut](/assets/img/emoji_key_shortcut.png)
    
- Removing Emoji Shortcut key is complete to use ‘Split Vertical’ shortcut in Terminator

    ![Del_Emoji_key](/assets/img/complete_del_emoji_key.png)

## 3. Terminator 설정 파일 세부 수정
- 추가적으로 Terminator 의 세부적인 설정을 변경하려면 아래의 경로의 config 파일을 열어 원하는 옵션을 변경한다

```bash
gedit ~/.config/terminator/config
```

> Terminator Config File (~/.config/terminator/config)

```bash
[global_config]
  handle_size = 0
  focus = system
[keybindings]
[layouts]
  [[default]]
    [[[child1]]]
      parent = window0
      type = Terminal
    [[[window0]]]
      parent = ""
      size = 1200, 600
      type = Window
[plugins]
[profiles]
  [[default]]
    scrollbar_position = hidden
    scrollback_infinite = True
    use_system_font = False
    background_darkness = 0.9
    background_type = transparent
    background_image = None
    show_titlebar = False
    font = D2Coding 12
```

## 4. Terminator 기본 단축키 사용법**

- 위/아래로 화면 나누기 :  Ctrl +  Shift + O
- 좌/우로 화면 나누기 :  Ctrl + Shift + E
- 현재 화면 닫기 : Ctrl + Shift + W
- 화면간 이동 : Alt + 방향키
- 스크롤바 Toggle : Ctrl + Shift + S
- 검색 :  Ctrl + Shift + F
- 화면 Clear :  Ctrl + Shift + G