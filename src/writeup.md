# Notes on Implementation

- Used twiddle to optimize PID parameters:
  - Kp: 1.1
  - Kd: 5.87422
  - Ki: 0
- Interestingly, Twiddle optimized the integral term to 0. I am going to explore more and find out why that is. This could be causing some of the jerkey behavior when steering.
- These are around the values I expected from what I saw when manually tuning.
  - When Kp was much higher than 1, the path oscilated too much.
  - Kd generally had to be higher than Kp to smooth out steering.
  - Ki usually had to be very small to avoid massive oversteering.
- Started twiddle steering PID at { 0.1, 0, 0 } to give a bit of a performance boost over { 0, 0, 0 }.
