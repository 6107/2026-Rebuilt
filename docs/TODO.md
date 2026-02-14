# Here are some must haves

Ordered by approximate priority. Moved to bottom (in order completed).

|             Status              |      Task       | Description                                                                                                                                                                                                   |
|:-------------------------------:|:---------------:|:--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|     <input type="checkbox">     |     CAN ID      | Get real drivetrain working with initial values and all CAN bus IDs assigne                                                                                                                                   |
|     <input type="checkbox">     |   Drivetrain    | Get it actually working and driveable by anyone                                                                                                                                                               |
|     <input type="checkbox">     | SPEED Constants | Figure out actual MAX_SPEED and MAX_ANGULAR_SPEED.<br/>Done when we know actual weight and test it.                                                                                                           |
|     <input type="checkbox">     |     Vision      | Get vision working  (Aim at tag, follow tag around)<br/>Focus on Limelight first. Photonvision is a second option but may need more work.                                                                     |
|     <input type="checkbox">     |     Shooter     | Basic subsystem support and proper pykit logging                                                                                                                                                              |
|     <input type="checkbox">     |     Intake      | Basic subsystem support and proper pykit logging                                                                                                                                                              |
|     <input type="checkbox">     |     Indexer     | Basic subsystem support and proper pykit logging                                                                                                                                                              |
|     <input type="checkbox">     |        .        | .                                                                                                                                                                                                             |
|     <input type="checkbox">     |   Controller    | Determine all the command mappings to the controllers.<br/>Make a PowerPoint set of slides that we can update as needed                                                                                       |
|     <input type="checkbox">     | SysId Routines  | Look into SysID Support and evaluate if we need it.<br/> Need good closed-loop parameter.                                                                                                                     |
|     <input type="checkbox">     | AUTO-Path Cmds  | Implement multiple possible auto-path commands.<br/>Need to come up with a good coordinate and base/fixed paths that we can pull together.                                                                    |
|     <input type="checkbox">     |   Aim & Shoot   | Get auto aim and shoot command implemented                                                                                                                                                                    |
|     <input type="checkbox">     | Shoot Postition | Come up with more than one shooting position. Best if it could be an algorythm                                                                                                                                |
|     <input type="checkbox">     |     Vision      | Add vision Logging Support                                                                                                                                                                                    |
|     <input type="checkbox">     |  Field Limits   | Support Field Limits in the 'ResetXY' command.<br/>Need to confirm that this is the center of our robot and do appropriate checks on space                                                                    |
|     <input type="checkbox">     |  Auto Recover   | Need an automated command that will get us off of a ball if in auto or teleop we get trapped.<br/>Will rely upon lots of testing to figure out a good mechanism.                                              |
|     <input type="checkbox">     | Drive function  | For Drive function, look into the 'square' and 'rate_limit' optional parameters                                                                                                                               |
|     <input type="checkbox">     |    Overshoot    | Test for odometry overshoot for yaw and see how best to adjust or compensate for it.<br/>Will need to be done when robot is complete empty and with fuel load in it                                           |
|     <input type="checkbox">     |  View Support   | Re-examine all drive and command code and make sure we have FieldCentric and RobotCentric properly set or compensated for                                                                                     |
|     <input type="checkbox">     |   Rev Client    | For the shooter, there is a Rev Client that may help us parameterize the values for the closed loop controller.                                                                                               |
|     <input type="checkbox">     |    Auto Max     | Do we need a different Autonomous MAX speed?                                                                                                                                                                  |
|     <input type="checkbox">     |    GotoPoint    | Test 'GotoPoint' command. see if we can tie it into pathplanner and have it traverse from the neutral zone to our alliance zone to shoot                                                                      |
|     <input type="checkbox">     |     Climber     | Basic subsystem support and proper pykit logging                                                                                                                                                              |
|     <input type="checkbox">     |        .        | .                                                                                                                                                                                                             |
|     <input type="checkbox">     |       PTP       | If using PhotonVision, verify or implement [Ping/Pong PhotonVision](https://docs.photonvision.org/en/latest/docs/contributing/design-descriptions/time-sync.html) workaround for accurate timestamps.
|     <input type="checkbox">     |        .        | .                                                                                                                                                                                                             |
|     <input type="checkbox">     |        .        | .                                                                                                                                                                                                             |
|     <input type="checkbox">     |     Cleanup     | Once pykit logging works. Drop our 'stats' capability                                                                                                                                                         |
|     <input type="checkbox">     |     Cleanup     | Once pykit ogging works, re-evaluate use of python logging.<br/>Will probably keep it, but need to minimize overhead and make messages worthwhile                                                             |
|     <input type="checkbox">     |     Cleanup     | Is the HolonomicDrive class used anywhere?   Clean up or get rid of it                                                                                                                                        |
|            **DONE**             |  -------------  | ---------------------------------------------                                                                                                                                                                 |
| <input type="checkbox" checked> |  Pitch & Roll   | Do we want to support pitch and roll in our logged gyro outputs.<br/>Seems like this would be useful since we have a bump and may help us figure out how to get off a ball underneath                         |

# Helpful items

Here are some things that are outstanding (but not needed or may not have the time).
No particular priority order.

|              Status             |      Task      | Description                                                                                                |
|:-------------------------------:|:--------------:|:-----------------------------------------------------------------------------------------------------------|
|     <input type="checkbox">     | AdvantageScope | Can read real logs                                                                                         |
|     <input type="checkbox">     | AdvantageScope | Can read sim logs                                                                                          |
|     <input type="checkbox">     | AdvantageScope | Can work with running robot (live)                                                                         |
|     <input type="checkbox">     | AdvantageScope | Add mechanism support - Shooter                                                                            |
|     <input type="checkbox">     | AdvantageScope | Can do replay-simulation with 2d and/or 3d of real or sim logs                                             |
|     <input type="checkbox">     | AdvantageScope | Augmented Reality works                                                                                    |
|     <input type="checkbox">     | AdvantageScope | Real image of our robot                                                                                    |
|     <input type="checkbox">     | AdvantageScope | Add mechanism support - Shooter                                                                            |
|     <input type="checkbox">     | AdvantageScope | Add mechanism support - Intake                                                                             |
|     <input type="checkbox">     | AdvantageScope | Add mechanism support - Indexer                                                                            |
|     <input type="checkbox">     | AdvantageScope | Add mechanism support - Climber                                                                            |
|     <input type="checkbox">     |     Visio      | Add multi-camera per VisionSubsystem support                                                                                                         |
|     <input type="checkbox">     |    Vibrate     | Vibrate controller feedback?                                                                               |
|     <input type="checkbox">     |     Music      | Good sounds for different commands (operations)                                                            |
|     <input type="checkbox">     |    Choosers    | LoggedDashboardChooser is provided by pykit. Change all Sendable Choosers to use this                      |
|     <input type="checkbox">     | SwerveToPoint  | After GotoPoint is understood. Does SwerveToPoint provide a better way if we combine that with pathplanner |
|     <input type="checkbox">     | field selector | For field selector, make that chooser work and update the fields. This is future of course                 |
|     <input type="checkbox">     |       .        | .                                                                                                          |
|     <input type="checkbox">     |       .        | .                                                                                                          |
|             **DONE**            | -------------  | ---------------------------------------------                                                              |
| <input type="checkbox" checked> | AdvantageScope | 3D Sim works                                                                                               |
| <input type="checkbox" checked> | AdvantageScope | Can work with simulated robot (live)                                                                       |

# Other

Once pykit is mostly working, look at the following two items and determine impact. These are High Priority.

- Westwood project has a 'drive-with-path-planned' that works with path planner and will log the chassis speed each time
  called. Determine if this is critical for pykit integration

- For the default pykit w/ pathplanner, it had a k_pathplanner_holonomic_controller and there was a PPTLVController
  defined. Did not see much mention in other projects about this. Look to see what it does and how it could help

