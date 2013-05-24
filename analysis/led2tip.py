import numpy as np

led_pos = np.array([0.706909511404, -0.0480302411391, 0.0385864020082])
tip_pos = np.array([.5936, -.0715, -.0261])

def get_tip_pos_from_led_pos(pos):
    return pos - led_pos + tip_pos

