import tkinter as tk
from tkinter import Button, Toplevel, ttk
import math
import json
import pandas as pd
import os
from functools import partial

root = tk.Tk()
root.title("Device Position Confirguration Interface")

dev_button_list = dict()
CONFIG_FILE_NAME = "/home/ubuntu/FYP-ROS/src/commander/commander/dev_config.json"
DEFAULT_DEV_NAME = "dev"
DEVICE_TYPES = ["LED", "PowerPoint"]

# TODO: deal with issue where device has same name or same IP


class InterfaceCanvas():
    def __init__(self):
        self.width = 600
        self.height = 600
        self.canvas = tk.Canvas(root, width=self.width, height=self.height)
        border = self.canvas.create_oval(self.width / 3, self.height / 3,
                                         self.width * 2 / 3, self.height * 2 / 3)
        centre = self.canvas.create_oval(self.width * 6 / 13, self.height * 6 / 13,
                                         self.width * 7 / 13, self.height * 7 / 13, fill="red")
        self.centre_x = self.width / 2
        self.centre_y = self.height / 2
        self.radius = self.width / 6
        self.add_labeled_text()
        add_btn = Button(root, text="Add device", width=19, height=2,
                         bd="10", command=self.create_new_device)
        add_btn.place(x=70, y=80)

        remove_btn = Button(root, text="Remove device", width=19, height=2,
                            bd="10", command=self.remove_last_object)
        remove_btn.place(x=310, y=80)
        export_btn = Button(root, text="Export Configuration", width=45, height=2,
                            bd="10", command=self.export_dev_json)
        export_btn.place(x=70, y=450)
        self.load_dev_json()
        self.canvas.pack()

    def add_labeled_text(self):
        self.canvas.create_text(self.centre_x,
                                self.centre_y, text="Glove")
        self.canvas.create_text(
            self.centre_x, self.centre_y - self.radius - 30, text="North (0)")
        self.canvas.create_text(
            self.centre_x, self.centre_y + self.radius + 30, text="South (180)")
        self.canvas.create_text(
            self.centre_x + self.radius + 60, self.centre_y, text="East (90)")
        self.canvas.create_text(
            self.centre_x - self.radius - 60, self.centre_y, text="West (270)")

    def create_new_device(self, name=DEFAULT_DEV_NAME):
        if name == DEFAULT_DEV_NAME:
            name += str(len(dev_button_list.keys()))
        edit_dev_pair = partial(self.edit_dev_config, name)
        dev_btn = Button(self.canvas, text=name, width=2, height=2,
                         bd="0", command=edit_dev_pair)
        dev_button_list[name] = {"instance": dev_btn}
        dev_button_list[name]["device_type"] = DEVICE_TYPES[0]
        dev_button_list[name]["ip_address"] = "0.0.0.0"
        self.refresh_dev_bt_pos()

    def remove_last_object(self):
        global dev_button_list
        if len(dev_button_list) == 0:
            return
        dev_to_delete = list(dev_button_list.keys())[-1]
        dev_button_list[dev_to_delete]["instance"].destroy()
        dev_button_list.pop(dev_to_delete)
        self.refresh_dev_bt_pos()

    def edit_dev_config(self, dev_name):
        edit_window = Toplevel(root)
        edit_window.title("Editing " + dev_name)
        edit_window.geometry("300x180")

        # Device Name
        cur_row = 0
        tk.Label(edit_window, text="Device Name").grid(row=cur_row)
        device_name_entry = tk.Entry(edit_window)
        device_name_entry.grid(row=cur_row, column=1)
        device_name_entry.insert(tk.END, dev_name)

        # Device Type
        cur_row += 1
        tk.Label(edit_window, text="Device Type").grid(row=cur_row)
        device_type_combo_box = ttk.Combobox(
            edit_window, state="readonly", values=DEVICE_TYPES)
        device_type_combo_box.current(DEVICE_TYPES.index(
            dev_button_list[dev_name]["device_type"]))
        device_type_combo_box.grid(row=cur_row, column=1)

        # Lower Bound
        cur_row += 1
        tk.Label(edit_window, text="Start Angle").grid(row=cur_row)
        upper_bound_entry = tk.Entry(edit_window)
        upper_bound_entry.insert(
            tk.END, dev_button_list[dev_name]["upper_bound"])
        upper_bound_entry.grid(row=cur_row, column=1)

        # Upper Bound Bound
        cur_row += 1
        tk.Label(edit_window, text="End Angle").grid(row=cur_row)
        lower_bound_entry = tk.Entry(edit_window)
        lower_bound_entry.insert(
            tk.END, dev_button_list[dev_name]["lower_bound"])
        lower_bound_entry.grid(row=cur_row, column=1)

        cur_row += 1
        tk.Label(edit_window, text="IP Address").grid(row=cur_row)
        ip_address_entry = tk.Entry(edit_window)
        ip_address_entry.insert(
            tk.END, dev_button_list[dev_name]["ip_address"])
        ip_address_entry.grid(row=cur_row, column=1)

        # Save Button
        cur_row += 1
        save_partial = partial(self.save_dev_settings, edit_window, dev_name, device_name_entry,
                               device_type_combo_box, upper_bound_entry, lower_bound_entry, ip_address_entry)
        save_button = Button(edit_window, text="Save",
                             width=4, height=2, bd="0", command=save_partial)
        save_button.grid(row=cur_row, column=0)

        # Quit Button
        save_button = Button(edit_window, text="Quit",
                             width=4, height=2, bd="0", command=edit_window.destroy)
        save_button.grid(row=cur_row, column=1)

    def save_dev_settings(self, edit_window, dev_name, device_name_entry, device_type_combo_box, upper_bound_entry, lower_bound_entry, ip_address_entry):
        global dev_button_list
        dev_button_list[dev_name]["instance"].destroy()
        dev_button_list.pop(dev_name)
        # self.create_new_device(device_name_entry.get())
        print(f"{upper_bound_entry.get()=}, {lower_bound_entry.get()=}")
        dev_button_list[device_name_entry.get()] = {
            "upper_bound": upper_bound_entry.get(),
            "lower_bound": lower_bound_entry.get(),
            "device_type": device_type_combo_box.get(),
            "ip_address": ip_address_entry.get()
        }
        edit_window.destroy()
        self.refresh_dev_bt_pos(auto_gen=False)

    def destroy_all_dev_btn(self):
        for index, btn in enumerate(dev_button_list.values()):
            if "instance" in btn:
                btn["instance"].destroy()
                btn.pop("instance")

    def refresh_dev_bt_pos(self, auto_gen=True):
        angle_divider = len(dev_button_list.keys())
        if angle_divider == 0:
            return
        angle_per_step = 360 / angle_divider
        self.destroy_all_dev_btn()

        for index, btn in enumerate(dev_button_list.values()):
            if auto_gen:
                cur_angle = angle_per_step * index
                btn["upper_bound"] = str(cur_angle + angle_per_step / 2)
                btn["lower_bound"] = str(
                    (cur_angle - angle_per_step / 2) % 360)
            else:
                cur_angle = (int(float(btn["upper_bound"])) +
                             int(float(btn["lower_bound"]))) / 2
            temp_name = list(dev_button_list.keys())[index]
            cur_x = self.centre_x - \
                math.sin(math.radians(-cur_angle)) * self.radius
            cur_y = self.centre_y - \
                math.cos(math.radians(cur_angle)) * self.radius

            if "instance" not in btn:
                name = list(dev_button_list.keys())[index]
                edit_dev_pair = partial(self.edit_dev_config, name)
                btn["instance"] = Button(self.canvas, text=name, width=2, height=2,
                                         bd="0", command=edit_dev_pair)
            self.place_dev_btn(btn["instance"], cur_x, cur_y)
        print()

    def place_dev_btn(self, instance, tar_x, tar_y):
        instance.place(x=tar_x-24, y=tar_y-24)

    def export_dev_json(self):
        df = pd.DataFrame(dev_button_list).T
        try:
            df = df[["upper_bound", "lower_bound",
                     "device_type", "ip_address"]].T
        except:
            print("No items so no column names")
        df.to_json(CONFIG_FILE_NAME)

    def load_dev_json(self):
        global dev_button_list
        files = os.listdir()
        if CONFIG_FILE_NAME not in files:
            return
        with open(CONFIG_FILE_NAME) as fp:
            dev_button_list = json.load(fp)
        self.refresh_dev_bt_pos()


if __name__ == "__main__":
    inteface = InterfaceCanvas()
    root.mainloop()
