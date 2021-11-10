# es_work
The C file drives a motor using a controlled signal from a joystick. Tuning the joystick, you can change the polarity and speed of the motor. 
This is implemented by an ADC that converts the simulated signal from joystick to digital data. Then we use digital data to control the duty cycle of the output wave. 

The vhdl files define the behavior of avalon bus slave, mainly realizing a real time clock.
