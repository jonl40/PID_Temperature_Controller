# PID_Temperature_Controller
C/C++ multi-threaded code for PID temperature controller and logger with Teensy 4.1 microcontroller 

P = Proportional (How fast the system responds)  
I = Integral (Used to remove steady state error)   
D = Derivative (Used to speed up the response of the system)

# Schematic 
![Schematic_PID](https://user-images.githubusercontent.com/33404359/134761129-6b3baf4b-a669-450f-bf81-f35e606b60c2.png)  

# PID Tuning recommendations:   
Derivative can add noise to the system   
Set I and D to zero then tune P  
Tune I when satisfied with P controllers response 
D can be left at 0 if a PI controller is suitable for your needs 


# References / Libraries Used:  

## Phil’s Lab PID Implementation:  
https://www.youtube.com/watch?v=zOByx3Izf5U  
https://github.com/pms67/PID  

## Multithreading library:   
Multithreading library by Fernando Trias https://github.com/ftrias/TeensyThreads  

## RTD Sensor library:   
Adafruit PT100/P1000 RTD Sensor w/MAX31865 library by Limor Fried/Ladyada @ Adafruit Industries  

## PID Tuning video:  
https://www.youtube.com/watch?v=IB1Ir4oCP5k  
