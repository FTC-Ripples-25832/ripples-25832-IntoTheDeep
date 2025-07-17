# NEED TO
# remove all dash stuff during actual match



# Robot logs (download from http://192.168.43.1:8080/logs)


https://www.youtube.com/watch?v=_R3X-8mXKnQ
https://www.youtube.com/watch?v=vvE_8sMhgd0
https://www.youtube.com/watch?v=ZligJsjWXOQ


2. test new dashboard limelight plugin

## TEST LOOP TIMES


limit lowerslide +claw to extend to 45cm from robot front. 17.7 inches
software limit for robot inspection part or else disqualified remember

3. finish auto


one mat tile is 24 inch
forward push test: 4*24=96 inches    ticks: 90095.0 = 0.000532770964
kV: 0.0000973235064406791, kS: 1.1415046110439833
lateral in per tick: 0.00039811630682462415
track width: 20701.1249475716
y-position: -9235.471589334393
y-position: 10013.518419847998
x-position: -2709.3228416164266
          public double par0YTicks = -9235.471589334393; // y position of the first parallel encoder (in tick
                                                               // units)
                public double par1YTicks = 10013.518419847998; // y position of the second parallel encoder (in tick
                // units)
                public double perpXTicks = -2709.3228416164266; // x position of the perpendicular encoder (in tick
                // units)


---



PIDF
https://www.youtube.com/watch?v=E6H6Nqe6qJo




- rename based on document
https://docs.google.com/document/d/1mU6lVJKcCrje3rY0VA7EhypkbPdXibo0Iwu1-oW6wVo/edit?usp=sharing


---

:8080/dash
:8030/dash
https://github.com/acmerobotics/ftc-dashboard

:8001
https://ftcontrol.bylazar.com/docs/plugins/core/limelight/
:5801 can also limelight if want separate


- more ftc dashboard, c720logi+huskylens supporttoo
        - draw robot, angles, field, paths to ftc dash
        https://github.com/FTCclueless/Centerstage/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utils/DashboardUtil.java





---

http://limelight.local:5801
https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
- fix limelight to claw
- fix limelight separate contours

https://www.chiefdelphi.com/t/ftc-12527-prototype-2025-build-thread/473681/32


---















backlog
---

https://pedropathing.com/overview.html
- if implement pure pursuit again yourself https://www.youtube.com/watch?v=tl9hurXdxCs
- pedro path use
https://github.com/6165-MSET-CuttleFish/pedro-path-generator
https://www.youtube.com/watch?v=HI7eyLLpCgM




---

team wesbite
find ones that look professional
copy
update content
add blog/docs/media/customstuff


---

ftc simulator
https://github.com/Shenzhen-Robotics-Alliance/maple-sim/tree/main?tab=readme-ov-file


---

- remove imu from odo for loop speed, important cuz more accurate, u ideally want 150+ cycles per second
https://www.youtube.com/watch?v=ixsxDn_ddLE&t=145s
https://www.youtube.com/watch?v=XkmZq2jopT4

heading rn is fed after a few spins, understand both vids 
and try
https://github.com/FTCclueless/Centerstage/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/drive/localizers/Localizer.java
or
https://github.com/frc7787/FTC-Into-The-Deep/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/purepursuit/localization/ThreeDeadWheelLocalizer.java

1.5cm error acceptable

need iterations
add video stuff and init localizer in swerve


##### in TESTSpeed.JAVA
https://ftcontrol.bylazar.com/docs/extras/looptimer/
https://gm0.org/en/latest/docs/software/tutorials/bulk-reads.html
3) Increase loop speeds & use a good approximation

    The faster the loop speed the more close the delta is to the differential.

You can increase loop speeds in a few ways:

    Only call a hardware component 1 timer per loop/don’t call it if you don’t use the data.
    Use BulkReads to call multiple devices at the same time https://gm0.org/en/latest/docs/softwa...
    Use analog and digital equivalents to I2C sensors (especially avoid the REV 2m Distance Sensor).
    Only update motors when they need to be updated
    Generally a constant velocity arc approximation is good enough for the 30 seconds of auto (In a good system expect 0.5 in of error). Road runner already uses this naturally.
    Try to minimize strafing, it makes your auto faster and increases accuracy.

---




archive if i want to save
---