# cart pole animations

## software simulation

open `index.html` in a browser

the animation uses the same nonlinear dynamics lqr gain measurement noise and kalman filter values as the python and esp32 versions

the controller only receives the kalman estimate while the animation also shows the hidden actual state for comparison

all calculations on this page run in browser javascript so it does not prove esp32 execution

## esp32 hardware in the loop

serve this folder from localhost and open `esp32.html` in desktop chrome or edge

for example from the project root

```text
python -m http.server 8000
```

then open

```text
http://localhost:8000/animation/esp32.html
```

click connect esp32 and select the esp32 serial port

the browser simulates only the nonlinear plant and noisy sensors then sends `MEAS,position,angle` to the esp32

the esp32 runs the kalman filter and lqr then returns `DATA,p,pdot,theta,thetadot,u`

restart run closes and reopens the selected port so the esp32 and browser plant both start cleanly

controls

- pause and reset the simulation
- push the cart left or right
- turn the lqr controller on or off
- choose a reset angle up to 45 degrees including 37.5
- change the maximum controller force from 1 to 30 N
- change the simulation speed
