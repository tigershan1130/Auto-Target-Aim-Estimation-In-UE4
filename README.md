# Auto-Target-Aim-Estimation-In-UE4
Using Kalman Filter/Least Square to Estimate Target to aim ahead.

The plugin includes:
- Perlin Noise added on real position to Fake Measurement Position, you can adjust frequency to slow down the offset.
- KalmanFilter and Least Square Regression algorithm to esimate target position ahead based on Fake Measurement Position.

Measurement-Position: Red Point
Kalman Filter: Blue Point
Least Square Estimation: Green Point

![alt text](https://github.com/tigershan1130/Auto-Target-Aim-Estimation-In-UE4/blob/master/ScreenShot01.jpg)


Warning: If you ran into turret not rotate properly, please adjust the Up Vector in bp.
