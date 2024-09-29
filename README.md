# PidController

For the Linux version, to build, run "source build.sh" from root dir, and to run, run "./build/src/pidProject" from root dir.

I ended up creating two projects: one in Linux and one in Windows. I tried first to do the project in Linux because I figured it was more applicable, and because my personal project laptop runs Linux. I was able to get the temperature reading successfully and wrote the PID code, but I got stuck on trying to get the LabJack to properly control the duty cycle. I was trying to use the basic LabJack exodriver functions and was unable to read from the device successfully. The code that is committed to the repo gives a failure to read error, but in previous experiments I was able to read a few bytes, but then would error saying that I was not able to read the full desired read. 

I moved to my wife's old Windows laptop and used the LabJack UD driver, which worked much better. Here I was able to control the LabJack properly, output the PWM signal, adjust the duty cycle correctly, and observe the temperature rise when I set the power high or remain stable when I set power low/zero. Unfortunately the thermometer would consistently stop being able to read at a little over 31 degrees Celsius. This was very consistent. I would start the program, watch it rise, and then recieve a "Bytes read do not match bytes requested" error as soon as it got to approximately 31.5 degrees. Then I would wait for ten minutes, allowing it to cool down, and then be able to run the experiment again. I looked for programmed limits for the CP2112 hardware, but was unable to find any. Unfortunately this made fine tuning the PID algorithm difficult, since I'd have to have large pauses in between debugging attempts, so there may still be issues with some of the values of the terms.

Overall, this project has been fun and I've definitely learned a lot! Thanks again for the opportunity!

