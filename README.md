# RoboVacuum
Project which aims to use a raspberry pi to control a robot vacuum cleaner. The base chasis is a Roboclean next generation, there are many generic robots using the same model. The majority of electronics were stripped and replaced with the Raspbery Pi 3 and some i2c hardware.

![Robo Clean](images/robo_clean.jpg?raw=true "Roboclean next generation")

After disassembly via removing all screws on bottom of robot all cirtcuit boards should be removed. From the "main" circuit board the contacts for the dustbin need to be cut off and stored.

![Robo Clean](images/original_guts.jpg?raw=true "Roboclean next generation")

Structural modification are detailed bellow:

The 5000mAh battery is about the same volume as the original 2400mAh. Unfurtunaly it i not the same in dimenion, so a section of the battery bay needs to be removed.

![Robo Clean](images/structure/battery_cut1.jpg?raw=true "Roboclean next generation")

Also the square pannel to the right needs to be removed.

![Robo Clean](images/structure/battery_cut2.jpg?raw=true "Roboclean next generation")

In the lid there are two modifications, a large rectangular hole should be cut for the RPI and two small slots for slide switches. The location of the slide switch holes are not important, but the chosen location is close to the battery terminals.

![Robo Clean](images/structure/lid_cut.jpg?raw=true "Roboclean next generation")