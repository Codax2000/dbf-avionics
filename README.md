# dbf-avionics
code for avionics package for RC planes.

This code was originally written for UW Design, Build, Fly by me, working with Noah Wager.
The purpose is to take input in from an RC receiver, read it efficiently, and upload it to an Arduino Nano, in addition to taking in
flight information from a pitot tube and onboard gyroscope/accelerometer. We used an MPU6050, a popular and common small GPU.

This is done using multiple libraries, which are listed here-special thanks to the people who wrote these.

I2Cdev and MPU6050, from his incredible i2cdevlib repo - https://github.com/jrowberg/i2cdevlib

EnableInterrupt, from GreyGnome - https://github.com/GreyGnome/EnableInterrupt

The eventual goal of this repository is to implement active stabilization, so that if a gust of wind hits the plane, it will resist
rolling. This will take a test flight to see what kind of rolling needs to be allowed, and what kind of calibration it would take.

Consider basing it off the photoelectric effect - until a certain roll speed, there is no accounting for roll. After that, accounting for
the roll takes place in a linear fashion.
