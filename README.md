# pwir2020_Madmax_Electronics-
## Functionality Discription 
    This electronic board consists of main control circuit and driver circuit.This circuit capable of driver three DC Gear Motors (16V) and communicate with on board computer via USB connection excute Motor rotation given speed and give feedback of speed.Another functionality of this board is controling thrower servo Motor speed.Driver Motor speed control using PI controller implement in firmware to archive desired speed quickly 
## Communication 
Communication between main PC and control board happens in HEX values(16 bits).
 driver Motor speeds send between 0-100 , thrower Motors speed (0-1000) and end command
 
 Motor1_speed,Motor2_speed,Motor3_speed,Thrower_speed,command_end
 
 E.g. 64 00 64 00 64 00 64 00 AA AA

## Contection ##
- Micro USB  : Comunitcate with PC where you can get speeds of the speeds of the motors and send feedback
- J7         : STM32 programmer connector
- J5         : Driver Motor2 encoder conector
    -1 : GND
    -2 :+3.3V
    -3 : Encoder Signal line 2 for Motor1
    -4 : Encoder Signal line 1 for Motor1
             
- J6         : Driver Motor3 encoder conector
- J4         : Driver Motor1 encoder conector
- J3         : Thrower Motor Signal connector
- U2         : Driver Motor1 power connector
- U3         : Driver Motor2 power connector
- U4         : Driver Motor3 power connector
- '+'          : Driver Motor powerconnection (battery 16V)

    
