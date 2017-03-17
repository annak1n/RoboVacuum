# RoboVacuum
Project which aims to use a raspberry pi to control a robot vacuum cleaner. The base chasis is a Roboclean next generation, there are many generic robots using the same model. The majority of electronics were stripped and replaced with the Raspbery Pi 3 and some i2c hardware.

![Robo Clean](images/robo_clean.jpg?raw=true "Roboclean next generation")

After disassembly via removing all screws on bottom of robot all cirtcuit boards should be removed. From the "main" circuit board the contacts for the dustbin need to be cut off and stored.

![Robo Clean](images/original_guts.jpg?raw=true "Roboclean next generation")

## Structural modification:

The 5000mAh battery is about the same volume as the original 2400mAh. Unfurtunaly it i not the same in dimenion, so a section of the battery bay needs to be removed.

![Robo Clean](images/structure/battery_cut1.jpg?raw=true "Roboclean next generation")

Also the square pannel to the right needs to be removed.

![Robo Clean](images/structure/battery_cut2.jpg?raw=true "Roboclean next generation")

In the lid there are two modifications, a large rectangular hole should be cut for the RPI and two small slots for slide switches. The location of the slide switch holes are not important, but the chosen location is close to the battery terminals. The lid is made of a softer plastic compared to the chasis (PE?), driling holes and then cutting with a stanley knife was suffient. Also marking with a knife and bending until it breaks was useful for adjusting the square hole size.

![Robo Clean](images/structure/lid_cut.jpg?raw=true "Roboclean next generation")

FInally the front IR proximity sensor in the lid should be removed.

## Part lists:
- Roboclean
- raspberry PI 3 model B
- Adafruit BNO055
- 2 X Adafruit motor and stepper hat
- ADC PI Plus (ABE electonics)
- Sharp IR (10-80cm) distance sensor
- 3000mAh battery (JST-XH-2P) for RPI
- 5000mAh battery (Hyperion G5 50C 3S 5000mAh LiPo Battery) for motors
- PowerBoost 1000 Charger

## Fitting electronics
    After removing the main board and cutting all the holes the eletronics can now be added. This can be seperated into two stages, the lide and the chasis.
### Lid
The lid first needs to have the Sharp IR sensor added, in the place of the original fron sensor. There is a little bit of adjustment required to get it flush with the front. As the plastic is soft you can do this with pliers.

![Robo Clean](images/assebly/dist_sensor1.jpg?raw=true "Roboclean next generation")

![Robo Clean](images/assebly/dist_sensor2.jpg?raw=true "Roboclean next generation")

The storeed contacts from the main board need some wires soldered onto them and can be mounted again in the same possition.

![Robo Clean](images/assebly/dustbin_contact.jpg?raw=true "Roboclean next generation")