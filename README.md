# First Robotics Challenge 2026 - Team 6107    CyberJagzz

This is our final run-time code used at the 2026 Rocket City Regionals.

We played as a defense bot due to incomplete intake/shooter and no production
testing of the climber.

## Other notes of interest

There is a bug in the pathplanner (or its python module) where a divide by zero error occurs. This
seemed to be on paths that were 'Duplicated' and then re-assigned. Did not impact the runtime code,
but the AutoChooser only had the 'Do Nothing' default. I reworked many of the paths and got auto
working, but as a defense-bot, did not use it except for 1 match.

The first two matches we had a dead robot. This was from an exception in the pykit logging library
we were using and it was only throwing an exception when working with the FMS during a match. Never
had an issue in sim or in any type of practice. Current code has try/catch around the offending
areas and a 'HACK' comment. Pykit is very desirable, but not if it is not ready for production. The
module is in our **lib_6107** subdirectory that I plan to split off into our own python installable
module. If time allows, may also make it async with respect to I/O and fix the darn 'flash drive'
error you get when you run without a flash drive. Also need to put it under unit test and get
better coverage.

We missed our practice matches under the FMS due to a very strange swerve drive issue (right front). 
That drive had plagued us for a while and it turned out to be that it did NOT have to be inverted,
even though the 'right back' was inverted and it should have been also.

Vision for _PhotonVision_ was available, but turned off since it was untested and we had other issues
of higher priority to address.

Lastly, we are taking about 23 milliseconds (out of 20 mS allowed) to do our most of our periodic tasks. This did
not cause an issue, but the console messages are annoying (and add a small amount of delay itself). This
needs to be addressed in the pykit improvements and better review of what we log would be good.



  


