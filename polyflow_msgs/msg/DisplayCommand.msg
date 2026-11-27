# What a small robot display shows. Addressed to the display's hardware row (display_id), like
# JointCommand is to a joint. The display's adapter draws it, and animates it between commands
# (blinking, easing the gaze), so a node only says what it wants, not every frame.

builtin_interfaces/Time stamp

string display_id

# A face expression: neutral, happy, sad, angry, surprised, sleepy, or off (screen dark).
string expression

# Where the eyes look, each -1..1. +x is toward the viewer's right, +y toward the bottom of the screen.
float64 gaze_x
float64 gaze_y
