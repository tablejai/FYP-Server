from flask import Flask, request
import pyautogui
import json
import time

app = Flask(__name__)

# Using option as commmand with no mapping
ppt_command_map = [
    "option",  # "STATIC",
    "up",  # "SLIDE_UP",
    "down",  # "SLIDE_DOWN",
    "left",  # "SLIDE_LEFT",
    "right",  # "SLIDE_RIGHT",
    "b",  # "ZOOM_IN" ,
    "w",  # "ZOOM_OUT",
    "option",  # "HIGHLIGHT",
    "option",  # "ON_YES",
    "option",  # "OFF_NO",
    "option",  # "END",
]

ppt_dim = True

def ppt_control_state(command):
    global ppt_dim
    if command == "w" and ppt_dim:
        pyautogui.press("b")
        ppt_dim = False
    elif command == "b" and not ppt_dim:
        pyautogui.press("b")
        ppt_dim = True
    else:
        pyautogui.press(command)


@app.route('/', methods=["POST"])
def receive_command():
    # TODO: Fix the powerpoint toggle issue
    # ppt_control_state(ppt_command_map[json.loads(request.data)["command"]])
    command_received = json.loads(request.data)["command"]
    pyautogui_output = ppt_command_map[command_received]
    print("pyautogui output: ", pyautogui_output)
    ppt_control_state(pyautogui_output)
    return 0

app.run(host='0.0.0.0', port=7000, debug=True)