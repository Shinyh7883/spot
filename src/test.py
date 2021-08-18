from typing import List
from pynput.keyboard import Listener
from spot_mini_functions import *
motion = motion()

run = 1

dot = [55, 50, 170]
commend = [[dot, dot, 2],[dot, dot, 2],[dot, dot, 2],[dot, dot, 2]]


def on_press(key):
    if key == key.up:
        motion.foward(commend, run)

def on_release(key):
    if key == key.up:
        run = 2

with Listener(on_press=on_press, on_release=on_release) as listner:
    listner.join()

    #중간에 어찌 멈추나