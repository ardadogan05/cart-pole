# cart pole animation

open `index.html` in a browser

the animation uses the same nonlinear dynamics lqr gain measurement noise and kalman filter values as the python and esp32 versions

the controller only receives the kalman estimate while the animation also shows the hidden actual state for comparison

controls

- pause and reset the simulation
- push the cart left or right
- turn the lqr controller on or off
- choose a reset angle up to 45 degrees including 37.5
- change the maximum controller force from 1 to 30 N
- change the simulation speed
