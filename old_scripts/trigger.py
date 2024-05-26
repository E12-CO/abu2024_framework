import subprocess
from gpiozero import LED, Button
from time import sleep

state = 0
team_pa = ""
start_pa = ""

def button_callback(button_name):
    global state, team_pa, start_pa
    if state == 0:
        if button_name == 2:
            print("Select: red team")
            team_pa = "red"
            state = 1
            sleep(1)
        elif button_name == 3:
            print("Select: blue team")
            team_pa = "blue"
            state = 1
            sleep(1)
    elif state == 1:
        if button_name == 1:
            print("Select: start")
            start_pa = "start"
            state = 2
            sleep(1)
        elif button_name == 4:
            print("Select: retry")
            start_pa = "retry"
            state = 2
            sleep(1)

def main():
    global state, team_pa, start_pa
    #print("team_pa:", team_pa)
    #print("start_pa:", start_pa)
    start_button = Button(22, hold_time=4)
    red_team_button = Button(24, hold_time=4)
    blue_team_button = Button(23, hold_time=4)
    retry_button = Button(27, hold_time=4)

    start_button.when_pressed = lambda: button_callback(1)
    red_team_button.when_pressed = lambda: button_callback(2)
    blue_team_button.when_pressed = lambda: button_callback(3)
    retry_button.when_pressed = lambda: button_callback(4)
    red = LED(17)
    while state < 2:
        pass
    launch_command = ["ros2", "launch", "abu_framework", "mainlaunch.py", "start_param:=" + str(start_pa), "team_param:=" + str(team_pa)]
    process = subprocess.call(launch_command)
    red.on()

if __name__ == "__main__":
    main()

