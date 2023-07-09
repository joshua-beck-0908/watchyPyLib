import watchylib
import time

watchy = watchylib.Watchy()
disp = watchy.display
t = watchy.time

#img = disp.image(filename='robot_idx.bmp', x=0, y=0)
clockTime = f'{t.hour:02}:{t.minute:02}'
clock = disp.text(clockTime, align=disp.CENTRE, bg=True, scale=4)
disp.refresh()

while True:
    print(watchy.accel.acceleration)
    time.sleep(1)
while True:
    clockTime = f'{t.hour:02}:{t.minute:02}'
    clock.text = clockTime
    time.sleep(60)
    disp.refresh()
