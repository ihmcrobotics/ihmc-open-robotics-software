package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.DoorOpenDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.WalkThroughDoorBehavior.WalkThroughDoorBehaviorState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SleepBehavior;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.CurrentBehaviorStatus;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class WalkThroughDoorBehavior extends StateMachineBehavior<WalkThroughDoorBehaviorState>
{
   private final boolean DEBUG = true;
   private boolean isDoorOpen = false;

   public enum WalkThroughDoorBehaviorState
   {
      STOPPED,
      SETUP_ROBOT,
      //clear planar regions and look down, then up only come here after a failed plan
      SEARCHING_FOR_DOOR, //search for general door location
      //if distance to door < approach point location skip this step.
      //walk to general door location... point offseet from door approach location
      //search for door precise location
      WALKING_TO_PUSH_DOOR, //if failed jump to clear state
      PUSH_DOOR_POWER_STANCE, //if failed jump to clear state

      WALKING_TO_PULL_DOOR,
      PULL_DOOR_POWER_STANCE,
      SEARCHING_FOR_DOOR_FINAL, //search for door handle
      OPEN_PUSH_DOOR, // in paralelle run the detect open door behavior if open door is not detected, go back to search for door 
      OPEN_PULL_DOOR, // in paralelle run the detect open door behavior if open door is not detected, go back to search for door 

      SET_UP_ROBOT_FOR_DOOR_WALK,
      WAITING_FOR_USER_CONFIRMATION,
      WALK_THROUGH_DOOR,
      RESET_ROBOT,
      FAILED,
      DONE
   }

   //this is the predefined walk to points relative to the door reference frame, these should eventualy be replaced by a behavior that finds the best location to walk up to given an arm task space 
   private Vector3D32 pushDoorOffsetPoint1 = new Vector3D32(0.5f, -0.9f, 0f);
   private Vector3D32 pushDoorOffsetPoint2 = new Vector3D32(0.5f, -0.6f, 0f);

   private Vector3D32 pullDoorOffsetPoint1 = new Vector3D32(1.15f, 1.2f, 0f);
   private Vector3D32 pullDoorOffsetPoint2 = new Vector3D32(1.1f, 0.8f, 0f);

   //define some of the sub-behaviors that will be used that are specific to this behavior
   private final SearchForDoorBehavior searchForDoorBehavior;
   private final OpenPushDoorBehavior openPushDoorBehavior;
   private final OpenPullDoorBehavior openPullDoorBehavior;

   private final WalkToInteractableObjectBehavior walkToInteractableObjectBehavior;
   private final ResetRobotBehavior resetRobotBehavior;

   //this hold all the primitive behaviors that get used across most behaviors.
   private final AtlasPrimitiveActions atlasPrimitiveActions;
   private SleepBehavior sleepBehavior;
   //sends out a door location packet for use in debugging. not really necesary until the door is found from a behavior instead of the user supplying its location

   private final DoorOpenDetectorBehaviorService doorOpenDetectorBehaviorService;
   private final IHMCROS2Publisher<HeadTrajectoryMessage> headTrajectoryPublisher;
   private final HumanoidReferenceFrames referenceFrames;
   
   private final IHMCROS2Publisher<BehaviorStatusPacket> behaviorStatusPublisher;

   // If the goal is within this proximity, the robot won't turn towards the goal but will maintain it's initial orientation while walking
   private static final double proximityToGoalToMaintainOrientation = 1.5;

   public WalkThroughDoorBehavior(String robotName, String yoNamePrefix, ROS2Node ros2Node, YoDouble yoTime, YoBoolean yoDoubleSupport,
                                  FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames,
                                  WholeBodyControllerParameters wholeBodyControllerParameters, AtlasPrimitiveActions atlasPrimitiveActions,
                                  YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(robotName, "walkThroughDoorBehavior", WalkThroughDoorBehaviorState.class, yoTime, ros2Node);
      headTrajectoryPublisher = createPublisherForController(HeadTrajectoryMessage.class);
      this.referenceFrames = referenceFrames;
      doorOpenDetectorBehaviorService = new DoorOpenDetectorBehaviorService(robotName, yoNamePrefix + "DoorOpenService", ros2Node, yoGraphicsListRegistry);
      //      doorOpenDetectorBehaviorService.setTargetIDToLocate(50);
      //      doorOpenDetectorBehaviorService.setExpectedFiducialSize(0.2032);
      registry.addChild(doorOpenDetectorBehaviorService.getYoVariableRegistry());
      addBehaviorService(doorOpenDetectorBehaviorService);

      sleepBehavior = new SleepBehavior(robotName, ros2Node, yoTime);

      this.atlasPrimitiveActions = atlasPrimitiveActions;
      //    basicTimingBehavior = new BasicTimingBehavior(robotName, ros2Node);
      //set up behaviors
      
      
      
      searchForDoorBehavior = new SearchForDoorBehavior(robotName, yoNamePrefix, ros2Node, yoTime, referenceFrames, atlasPrimitiveActions);
      walkToInteractableObjectBehavior = new WalkToInteractableObjectBehavior(robotName, yoTime, ros2Node, atlasPrimitiveActions);

      openPushDoorBehavior = new OpenPushDoorBehavior(robotName,
                                                      yoNamePrefix,
                                                      yoTime,
                                                      ros2Node,
                                                      atlasPrimitiveActions,
                                                      doorOpenDetectorBehaviorService,
                                                      yoGraphicsListRegistry);
      openPullDoorBehavior = new OpenPullDoorBehavior(robotName,
                                                      yoNamePrefix,
                                                      yoTime,
                                                      ros2Node,
                                                      atlasPrimitiveActions,
                                                      doorOpenDetectorBehaviorService,referenceFrames,
                                                      yoGraphicsListRegistry);
      resetRobotBehavior = new ResetRobotBehavior(robotName, ros2Node, yoTime);
      
      ROS2Topic outputTopic = IHMCHumanoidBehaviorManager.getOutputTopic(robotName);
      behaviorStatusPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, BehaviorStatusPacket.class, outputTopic);

      setupStateMachine();
   }

   @Override
   public void doControl()
   {

      //should constantly be searching for door and updating its location here

      if (doorOpenDetectorBehaviorService.newPose != null)
      {

         Point3D location = new Point3D();
         Quaternion orientation = new Quaternion();
         doorOpenDetectorBehaviorService.newPose.get(location, orientation);
         publishUIPositionCheckerPacket(location, orientation);
      }

      if (isDoorOpen != doorOpenDetectorBehaviorService.isDoorOpen())
      {
         isDoorOpen = doorOpenDetectorBehaviorService.isDoorOpen();
         if (isDoorOpen)
            publishTextToSpeech("Door is Open");

         else
            publishTextToSpeech("Door is Closed");
      }

      super.doControl();

   }

   @Override
   public void onBehaviorEntered()
   {
      publishTextToSpeech("Entering Walk Through Door behavior");

      super.onBehaviorEntered();
   }

   @Override
   protected WalkThroughDoorBehaviorState configureStateMachineAndReturnInitialKey(StateMachineFactory<WalkThroughDoorBehaviorState, BehaviorAction> factory)
   {
      //reset the robot in case it is in a wierd configuration before the behavior starts
      BehaviorAction resetRobot = new BehaviorAction(resetRobotBehavior);

      //if there are hands, close them
      BehaviorAction setup = new BehaviorAction(atlasPrimitiveActions.leftHandDesiredConfigurationBehavior,
                                                atlasPrimitiveActions.rightHandDesiredConfigurationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            if (DEBUG)
            {
               publishTextToSpeech("entering setup");
            }
            HandDesiredConfigurationMessage leftHandMessage = HumanoidMessageTools.createHandDesiredConfigurationMessage(RobotSide.LEFT,
                                                                                                                         HandConfiguration.CLOSE);
            HandDesiredConfigurationMessage rightHandMessage = HumanoidMessageTools.createHandDesiredConfigurationMessage(RobotSide.RIGHT,
                                                                                                                          HandConfiguration.CLOSE);

            atlasPrimitiveActions.rightHandDesiredConfigurationBehavior.setInput(rightHandMessage);

            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior.setInput(leftHandMessage);

         }

      };

      //this is the first search for the door, once automated searching is in place, this should be an all the time thing.
      BehaviorAction searchForDoorFar = new BehaviorAction(searchForDoorBehavior)
      {
         
         @Override
         protected void setBehaviorInput()
         {
            searchForDoorBehavior.setScanForDoor(true);
            super.setBehaviorInput();
         }

         @Override
         public void onEntry()
         {
            publishTextToSpeech("Searching For The Door");
            //lookDown();
            super.onEntry();
         }

      };

      BehaviorAction searchForDoorNear = new BehaviorAction(searchForDoorBehavior)
      {

         @Override
         protected void setBehaviorInput()
         {
            searchForDoorBehavior.setScanForDoor(true);

            super.setBehaviorInput();
         }

         @Override
         public void onEntry()
         {
            publishTextToSpeech("Confirm door location before trying to open");

            super.onEntry();
         }

         @Override
         public void onExit()
         {

            System.out.println("SETTING OPEN DOOR ACTION INPUT " + searchForDoorBehavior.getLocation());

            super.onExit();
         }
      };

      BehaviorAction walkToDoorPushDoorAction = new BehaviorAction(walkToInteractableObjectBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            lookDown();
            if (DEBUG)
            {
               publishTextToSpeech("walk to push door action");
            }
            FramePoint3D point1 = offsetPointFromDoor(pushDoorOffsetPoint1);
            FramePoint3D point2 = offsetPointFromDoor(pushDoorOffsetPoint2);

            walkToInteractableObjectBehavior.setProximityToGoalToKeepOrientation(proximityToGoalToMaintainOrientation);
            walkToInteractableObjectBehavior.setWalkPoints(point1, point2);
         }
      };

      BehaviorAction walkToDoorPullDoorAction = new BehaviorAction(walkToInteractableObjectBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            lookDown();
            if (DEBUG)
            {
               publishTextToSpeech("walk to pull door action");
            }
            FramePoint3D point1 = offsetPointFromDoor(pullDoorOffsetPoint1);
            FramePoint3D point2 = offsetPointFromDoor(pullDoorOffsetPoint2);

            walkToInteractableObjectBehavior.setProximityToGoalToKeepOrientation(proximityToGoalToMaintainOrientation);
            walkToInteractableObjectBehavior.setWalkPoints(point1, point2);
         }
      };

      BehaviorAction openPushDoorAction = new BehaviorAction(openPushDoorBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            lookDown();
            if (DEBUG)
            {
               publishTextToSpeech("open door action");
            }
            //this should happen in the openDoorBehavior
            openPushDoorBehavior.setGrabLocation(searchForDoorBehavior.getLocation());
         }
      };

      BehaviorAction openPullDoorAction = new BehaviorAction(openPullDoorBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            lookDown();
            if (DEBUG)
            {
               publishTextToSpeech("open door action");
            }
            //this should happen in the openDoorBehavior
            openPullDoorBehavior.setGrabLocation(searchForDoorBehavior.getLocation());
         }
      };

      BehaviorAction setUpForWalk = new BehaviorAction(atlasPrimitiveActions.leftArmTrajectoryBehavior, atlasPrimitiveActions.rightArmTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            if (DEBUG)
            {
               publishTextToSpeech("setup for walk");
            }
            //            double[] rightArmPose = new double[] {1.5708, 0.8226007082651046, 1.2241049170121854, -1.546127437107859, -0.8486641166791746, -1.3365746544030488,
            //                  1.3376930879072813};
            //            double[] leftArmPose = new double[] {-1.5383305366909918, -0.9340404711083553, 1.9634792241521146, 0.9236260708644913, -0.8710518130931819,
            //                  -0.8771109242461594, -1.336089159719967};

            //          double[] rightArmPose = new double[] {1.5708, 0.46659982767419184, 1.304246503693704, -2.145974311961388, -1.4042857563420477, -1.1630658351411727, 2.0735697767004733};
            //          double[] leftArmPose = new double[] {-1.4573707384127728, -0.8679045375063197, 1.7161679382174408, 1.5565002230277143, -0.1665184664421956, -1.2894759927886323, -2.5972820987306298};

            double[] rightArmPose = new double[] {1.4183220415980604, 0.39055667789847864, 0.9442635783301307, -2.0569246166596393, 2.8812595031701362,
                  -0.06984858968272295, 0.7887047653418846};
            double[] leftArmPose = new double[] {-1.4141618751532627, -1.0073892313065864, 1.879864339482996, 1.461406972715835, 0.028921330852778603,
                  -1.1543763213792428, -2.797065870312528};

            ArmTrajectoryMessage rightPoseMessage = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.RIGHT, 4, rightArmPose);

            ArmTrajectoryMessage leftPoseMessage = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.LEFT, 4, leftArmPose);

            atlasPrimitiveActions.leftArmTrajectoryBehavior.setInput(leftPoseMessage);
            atlasPrimitiveActions.rightArmTrajectoryBehavior.setInput(rightPoseMessage);
         }
      };

      BehaviorAction pushPowerStance = new BehaviorAction(atlasPrimitiveActions.footstepListBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            if (DEBUG)
            {
               doorOpenDetectorBehaviorService.run(false);
               publishTextToSpeech("walk through door action");
            }

            PoseReferenceFrame doorPose = new PoseReferenceFrame("DoorReferenceFrame", ReferenceFrame.getWorldFrame());

            Pose3D unrotatedDoor = new Pose3D(searchForDoorBehavior.getLocation());

            double offsetLeftRight = 0;
            double footSpread = 0;
            FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(atlasPrimitiveActions.footstepListBehavior.getDefaultSwingTime(),
                                                                                                 atlasPrimitiveActions.footstepListBehavior.getDefaultTranferTime());

            offsetLeftRight = .381;
            footSpread = -.05;
            unrotatedDoor.appendYawRotation(Math.toRadians(180));
            unrotatedDoor.appendTranslation(-0.9144, 0, 0);

            doorPose.setPoseAndUpdate(unrotatedDoor);

            RobotSide startStep = RobotSide.LEFT;

            FootstepDataMessage fs1 = createRelativeFootStep(doorPose,
                                                             startStep.getOppositeSide(),
                                                             new Point3D(-0.124 + offsetLeftRight, pushDoorOffsetPoint2.getX() + 0.635, -0),
                                                             new Quaternion(-4.624094786785623E-5,
                                                                            3.113506928734585E-6,
                                                                            -0.7043244487834723,
                                                                            0.7098782069467541));

            message.getFootstepDataList().add().set(fs1);

            message.setTrustHeightOfFootsteps(true);

            atlasPrimitiveActions.footstepListBehavior.set(message);
         }
      };

      BehaviorAction pullPowerStance = new BehaviorAction(atlasPrimitiveActions.footstepListBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            publishTextToSpeech("Pull Door Power Stance");

            PoseReferenceFrame doorPose = new PoseReferenceFrame("DoorReferenceFrame", ReferenceFrame.getWorldFrame());

            Pose3D unrotatedDoor = new Pose3D(searchForDoorBehavior.getLocation());

            double offsetLeftRight = 0;
            double footSpread = 0;
            FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(atlasPrimitiveActions.footstepListBehavior.getDefaultSwingTime(),
                                                                                                 atlasPrimitiveActions.footstepListBehavior.getDefaultTranferTime());

            offsetLeftRight = .381;
            footSpread = -.05;
          //unrotatedDoor.appendYawRotation(Math.toRadians(180));
          //unrotatedDoor.appendTranslation(-0.9144, 0, 0);

            doorPose.setPoseAndUpdate(unrotatedDoor);

            RobotSide startStep = RobotSide.LEFT;

            FootstepDataMessage fs1 = createRelativeFootStep(doorPose,
                                                             startStep,
                                                             new Point3D(1.172, 0.602, 0.092),
                                                             new Quaternion(-4.624094786785623E-5,
                                                                            3.113506928734585E-6,
                                                                            -0.7043244487834723,
                                                                            0.7098782069467541));

            message.getFootstepDataList().add().set(fs1);

            message.setTrustHeightOfFootsteps(true);

            atlasPrimitiveActions.footstepListBehavior.set(message);
         }
      };

      BehaviorAction walkThroughDoor = new BehaviorAction(atlasPrimitiveActions.footstepListBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            lookUp();

            if (DEBUG)
            {
               doorOpenDetectorBehaviorService.run(false);
               publishTextToSpeech("walk through door action");
            }
            FootstepDataListMessage message = setUpFootSteps();
            atlasPrimitiveActions.footstepListBehavior.set(message);
         }
      };

      BehaviorAction failedState = new BehaviorAction(new SimpleDoNothingBehavior(robotName, ros2Node))
      {
         @Override
         protected void setBehaviorInput()
         {
            behaviorStatusPublisher.publish(HumanoidMessageTools.createBehaviorStatusPacket(CurrentBehaviorStatus.BEHAVIOR_FINISHED_FAILED, HumanoidBehaviorType.WALK_THROUGH_DOOR));
            publishTextToSpeech("Walking Through Door Failed");
         }
      };

      BehaviorAction doneState = new BehaviorAction(sleepBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            behaviorStatusPublisher.publish(HumanoidMessageTools.createBehaviorStatusPacket(CurrentBehaviorStatus.BEHAVIOR_FINISHED_SUCCESS, HumanoidBehaviorType.WALK_THROUGH_DOOR));
            sleepBehavior.setSleepTime(1);
            publishTextToSpeech("Finished Walking Through Door");
         }
      };

      factory.addStateAndDoneTransition(WalkThroughDoorBehaviorState.SETUP_ROBOT, resetRobot, WalkThroughDoorBehaviorState.SEARCHING_FOR_DOOR);
      factory.addState(WalkThroughDoorBehaviorState.SEARCHING_FOR_DOOR, searchForDoorFar);

      //PUSH ******************

      factory.addTransition(WalkThroughDoorBehaviorState.SEARCHING_FOR_DOOR,
                            WalkThroughDoorBehaviorState.WALKING_TO_PUSH_DOOR,
                            t -> searchForDoorFar.isDone() && searchForDoorBehavior.getDoorType() == DoorLocationPacket.PUSH_HANDLE_RIGHT);

      factory.addState(WalkThroughDoorBehaviorState.WALKING_TO_PUSH_DOOR, walkToDoorPushDoorAction);

      factory.addTransition(WalkThroughDoorBehaviorState.WALKING_TO_PUSH_DOOR,
                            WalkThroughDoorBehaviorState.SEARCHING_FOR_DOOR_FINAL,
                            t -> isWalkingDone() && hasWalkingSucceded());
      factory.addTransition(WalkThroughDoorBehaviorState.WALKING_TO_PUSH_DOOR,
                            WalkThroughDoorBehaviorState.FAILED,
                            t -> isWalkingDone() && !hasWalkingSucceded());

      //PULL ******************

      factory.addTransition(WalkThroughDoorBehaviorState.SEARCHING_FOR_DOOR,
                            WalkThroughDoorBehaviorState.WALKING_TO_PULL_DOOR,
                            t -> searchForDoorFar.isDone() && searchForDoorBehavior.getDoorType() == DoorLocationPacket.PULL_HANDLE_LEFT);

      factory.addState(WalkThroughDoorBehaviorState.WALKING_TO_PULL_DOOR, walkToDoorPullDoorAction);

      factory.addTransition(WalkThroughDoorBehaviorState.WALKING_TO_PULL_DOOR,
                            WalkThroughDoorBehaviorState.PULL_DOOR_POWER_STANCE,
                            t -> isWalkingDone() && hasWalkingSucceded());

      factory.addTransition(WalkThroughDoorBehaviorState.WALKING_TO_PULL_DOOR,
                            WalkThroughDoorBehaviorState.FAILED,
                            t -> isWalkingDone() && !hasWalkingSucceded());

      factory.addStateAndDoneTransition(WalkThroughDoorBehaviorState.PULL_DOOR_POWER_STANCE,
                                        pullPowerStance,
                                        WalkThroughDoorBehaviorState.SEARCHING_FOR_DOOR_FINAL);

      factory.addState(WalkThroughDoorBehaviorState.SEARCHING_FOR_DOOR_FINAL, searchForDoorNear);

      factory.addTransition(WalkThroughDoorBehaviorState.SEARCHING_FOR_DOOR_FINAL,
                            WalkThroughDoorBehaviorState.OPEN_PUSH_DOOR,
                            t -> searchForDoorNear.isDone() && searchForDoorBehavior.getDoorType() == DoorLocationPacket.PUSH_HANDLE_RIGHT);

      factory.addTransition(WalkThroughDoorBehaviorState.SEARCHING_FOR_DOOR_FINAL,
                            WalkThroughDoorBehaviorState.OPEN_PULL_DOOR,
                            t -> searchForDoorNear.isDone() && searchForDoorBehavior.getDoorType() == DoorLocationPacket.PULL_HANDLE_LEFT);

      //OPEN PUSH DOOR      
      factory.addState(WalkThroughDoorBehaviorState.OPEN_PUSH_DOOR, openPushDoorAction);

      factory.addTransition(WalkThroughDoorBehaviorState.OPEN_PUSH_DOOR,
                            WalkThroughDoorBehaviorState.SEARCHING_FOR_DOOR_FINAL,
                            t -> openPushDoorAction.isDone() && !openPushDoorBehavior.succeeded());
      factory.addTransition(WalkThroughDoorBehaviorState.OPEN_PUSH_DOOR,
                            WalkThroughDoorBehaviorState.SET_UP_ROBOT_FOR_DOOR_WALK,
                            t -> openPushDoorAction.isDone() && openPushDoorBehavior.succeeded());

      //OPEN PULL DOOR
      factory.addState(WalkThroughDoorBehaviorState.OPEN_PULL_DOOR, openPullDoorAction);

      factory.addTransition(WalkThroughDoorBehaviorState.OPEN_PULL_DOOR,
                            WalkThroughDoorBehaviorState.SEARCHING_FOR_DOOR_FINAL,
                            t -> openPullDoorAction.isDone() && !openPullDoorBehavior.succeeded());
      factory.addTransition(WalkThroughDoorBehaviorState.OPEN_PULL_DOOR,
                            WalkThroughDoorBehaviorState.SET_UP_ROBOT_FOR_DOOR_WALK,
                            t -> openPullDoorAction.isDone() && openPullDoorBehavior.succeeded());

      ///

      factory.addState(WalkThroughDoorBehaviorState.SET_UP_ROBOT_FOR_DOOR_WALK, setUpForWalk);

      //factory.addTransition(WalkThroughDoorBehaviorState.SET_UP_ROBOT_FOR_DOOR_WALK,
      //                      WalkThroughDoorBehaviorState.OPEN_PUSH_DOOR,
       //                     t -> setUpForWalk.isDone() && !doorOpenDetectorBehaviorService.isDoorOpen());
      factory.addTransition(WalkThroughDoorBehaviorState.SET_UP_ROBOT_FOR_DOOR_WALK,
                            WalkThroughDoorBehaviorState.WALK_THROUGH_DOOR,
                            t -> setUpForWalk.isDone());// && doorOpenDetectorBehaviorService.isDoorOpen());

      //factory.addStateAndDoneTransition(WalkThroughDoorBehaviorState.WALK_THROUGH_DOOR, walkThroughDoor, WalkThroughDoorBehaviorState.RESET_ROBOT);
      factory.addStateAndDoneTransition(WalkThroughDoorBehaviorState.WALK_THROUGH_DOOR, walkThroughDoor, WalkThroughDoorBehaviorState.DONE);

      factory.addStateAndDoneTransition(WalkThroughDoorBehaviorState.RESET_ROBOT, resetRobot, WalkThroughDoorBehaviorState.DONE);
      factory.addState(WalkThroughDoorBehaviorState.FAILED, failedState);
      factory.addState(WalkThroughDoorBehaviorState.DONE, doneState);

      factory.addStateChangedListener((from, to) ->
      {
         publishTextToSpeech((from == null ? null : from.name()) + " -> " + (to == null ? null : to.name()));
      });

      return WalkThroughDoorBehaviorState.SETUP_ROBOT;
   }

   private FramePoint3D offsetPointFromDoor(Vector3D32 point)
   {

      PoseReferenceFrame doorPose = new PoseReferenceFrame("doorFrame", ReferenceFrame.getWorldFrame());
      doorPose.setPoseAndUpdate(new Pose3D(searchForDoorBehavior.getLocation()));

      FramePoint3D point1 = new FramePoint3D(doorPose, point);
      return point1;
   }

   public FootstepDataListMessage setUpFootSteps()
   {

      PoseReferenceFrame doorPose = new PoseReferenceFrame("DoorReferenceFrame", ReferenceFrame.getWorldFrame());

      Pose3D unrotatedDoor = new Pose3D(searchForDoorBehavior.getLocation());

      double offsetLeftRight = 0;
      double footSpread = 0;
      FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(atlasPrimitiveActions.footstepListBehavior.getDefaultSwingTime(),
                                                                                           atlasPrimitiveActions.footstepListBehavior.getDefaultTranferTime());
      if (searchForDoorBehavior.getDoorType() == DoorLocationPacket.PUSH_HANDLE_RIGHT)
      {
         offsetLeftRight = .381;
         footSpread = -.05;
         unrotatedDoor.appendYawRotation(Math.toRadians(180));
         unrotatedDoor.appendTranslation(-0.9144, 0, 0);

         doorPose.setPoseAndUpdate(unrotatedDoor);

         RobotSide startStep = RobotSide.LEFT;

         FootstepDataMessage fs1 = createRelativeFootStep(doorPose,
                                                          startStep.getOppositeSide(),
                                                          new Point3D(-0.124 + offsetLeftRight, 0.592160790421584, -0),
                                                          new Quaternion(-4.624094786785623E-5, 3.113506928734585E-6, -0.7043244487834723, 0.7098782069467541));

         FootstepDataMessage fs2 = createRelativeFootStep(doorPose,
                                                          startStep,
                                                          new Point3D(0.124 + offsetLeftRight, 0.592160790421584, -0),
                                                          new Quaternion(-4.624094786785623E-5, 3.113506928734585E-6, -0.7043244487834723, 0.7098782069467541));

         FootstepDataMessage fs3 = createRelativeFootStep(doorPose,
                                                          startStep.getOppositeSide(),
                                                          new Point3D(footSpread + offsetLeftRight, 0.23597592988662308, -0),
                                                          new Quaternion(-1.5943418991263463E-13,
                                                                         2.75059506574629E-13,
                                                                         -0.7043243641759355,
                                                                         0.7098782924052293));

         FootstepDataMessage fs4 = createRelativeFootStep(doorPose,
                                                          startStep,
                                                          new Point3D(-footSpread + offsetLeftRight, -0.26851462759487155, -0),
                                                          new Quaternion(-3.236982396751798E-13,
                                                                         3.899712427026468E-14,
                                                                         -0.7043243760613419,
                                                                         0.7098782806128114));
         FootstepDataMessage fs5 = createRelativeFootStep(doorPose,
                                                          startStep.getOppositeSide(),
                                                          new Point3D(-0.124 + offsetLeftRight, -0.7234607322382425, -0),
                                                          new Quaternion(1.7351711631778928E-14,
                                                                         -1.6924263791365571E-13,
                                                                         -0.7043243760613419,
                                                                         0.7098782806128114));
         FootstepDataMessage fs6 = createRelativeFootStep(doorPose,
                                                          startStep,
                                                          new Point3D(0.124 + offsetLeftRight, -0.7199905519593679, -0),
                                                          new Quaternion(2.5501844493298926E-13,
                                                                         -3.0463423083022023E-13,
                                                                         -0.7043243760613419,
                                                                         0.7098782806128114));

         message.getFootstepDataList().add().set(fs1);
         message.getFootstepDataList().add().set(fs2);
         message.getFootstepDataList().add().set(fs3);
         message.getFootstepDataList().add().set(fs4);
         message.getFootstepDataList().add().set(fs5);
         message.getFootstepDataList().add().set(fs6);
         message.setTrustHeightOfFootsteps(true);
      }
      if (searchForDoorBehavior.getDoorType() == DoorLocationPacket.PULL_HANDLE_LEFT)
      {

         offsetLeftRight = .481;
         footSpread = -.05;

         doorPose.setPoseAndUpdate(unrotatedDoor);

         RobotSide startStep = RobotSide.LEFT;
         
         
         FootstepDataMessage fs1a = createRelativeFootStep(doorPose,
                                                           startStep.getOppositeSide(),
                                                           new Point3D(0.268 + offsetLeftRight, 0.9, -0),
                                                           new Quaternion(-4.624094786785623E-5,
                                                                          3.113506928734585E-6,
                                                                          -0.7043244487834723,
                                                                          0.7098782069467541));

         FootstepDataMessage fs2a = createRelativeFootStep(doorPose,
                                                           startStep,
                                                           new Point3D(0.506 + offsetLeftRight, 0.9, -0),
                                                           new Quaternion(-4.624094786785623E-5,
                                                                          3.113506928734585E-6,
                                                                          -0.7043244487834723,
                                                                          0.7098782069467541));
         

         FootstepDataMessage fs1p = createRelativeFootStep(doorPose,
                                                           startStep.getOppositeSide(),
                                                           new Point3D(0.148 + offsetLeftRight, 0.7, -0),
                                                           new Quaternion(-4.624094786785623E-5,
                                                                          3.113506928734585E-6,
                                                                          -0.7043244487834723,
                                                                          0.7098782069467541));

         FootstepDataMessage fs2p = createRelativeFootStep(doorPose,
                                                           startStep,
                                                           new Point3D(0.396 + offsetLeftRight, 0.7, -0),
                                                           new Quaternion(-4.624094786785623E-5,
                                                                          3.113506928734585E-6,
                                                                          -0.7043244487834723,
                                                                          0.7098782069467541));

         FootstepDataMessage fs1 = createRelativeFootStep(doorPose,
                                                          startStep.getOppositeSide(),
                                                          new Point3D(-0.124 + offsetLeftRight, 0.592160790421584, -0),
                                                          new Quaternion(-4.624094786785623E-5, 3.113506928734585E-6, -0.7043244487834723, 0.7098782069467541));

         FootstepDataMessage fs2 = createRelativeFootStep(doorPose,
                                                          startStep,
                                                          new Point3D(0.1 + offsetLeftRight, 0.592160790421584, -0),
                                                          new Quaternion(-4.624094786785623E-5, 3.113506928734585E-6, -0.7043244487834723, 0.7098782069467541));

         FootstepDataMessage fs3 = createRelativeFootStep(doorPose,
                                                          startStep.getOppositeSide(),
                                                          new Point3D(footSpread + offsetLeftRight, 0.23597592988662308, -0),
                                                          new Quaternion(-1.5943418991263463E-13,
                                                                         2.75059506574629E-13,
                                                                         -0.7043243641759355,
                                                                         0.7098782924052293));

         FootstepDataMessage fs4 = createRelativeFootStep(doorPose,
                                                          startStep,
                                                          new Point3D(-footSpread + offsetLeftRight, -0.26851462759487155, -0),
                                                          new Quaternion(-3.236982396751798E-13,
                                                                         3.899712427026468E-14,
                                                                         -0.7043243760613419,
                                                                         0.7098782806128114));
         FootstepDataMessage fs5 = createRelativeFootStep(doorPose,
                                                          startStep.getOppositeSide(),
                                                          new Point3D(-0.124 + offsetLeftRight, -0.7234607322382425, -0),
                                                          new Quaternion(1.7351711631778928E-14,
                                                                         -1.6924263791365571E-13,
                                                                         -0.7043243760613419,
                                                                         0.7098782806128114));
         FootstepDataMessage fs6 = createRelativeFootStep(doorPose,
                                                          startStep,
                                                          new Point3D(0.124 + offsetLeftRight, -0.7199905519593679, -0),
                                                          new Quaternion(2.5501844493298926E-13,
                                                                         -3.0463423083022023E-13,
                                                                         -0.7043243760613419,
                                                                         0.7098782806128114));
         message.getFootstepDataList().add().set(fs1a);
         message.getFootstepDataList().add().set(fs2a);
         message.getFootstepDataList().add().set(fs1p);
         message.getFootstepDataList().add().set(fs2p);
         message.getFootstepDataList().add().set(fs1);
         message.getFootstepDataList().add().set(fs2);
         message.getFootstepDataList().add().set(fs3);
         message.getFootstepDataList().add().set(fs4);
         message.getFootstepDataList().add().set(fs5);
         message.getFootstepDataList().add().set(fs6);
         message.setTrustHeightOfFootsteps(true);
      }

      return message;
   }

   private FootstepDataMessage createRelativeFootStep(PoseReferenceFrame frame, RobotSide side, Point3D location, Quaternion orientation)
   {

      FramePose3D pose = offsetPointFromFrameInWorldFrame(frame, location, orientation);
      double footZ2 = referenceFrames.getSoleFrame(RobotSide.LEFT).getTransformToWorldFrame().getTranslationZ();

      pose.setZ(footZ2);
      FootstepDataMessage message = HumanoidMessageTools.createFootstepDataMessage(side, pose.getPosition(), pose.getOrientation());
      return message;
   }

   private FramePose3D offsetPointFromFrameInWorldFrame(PoseReferenceFrame frame, Point3D point3d, Quaternion quat4d)
   {
      FramePoint3D point1 = new FramePoint3D(frame, point3d);
      point1.changeFrame(ReferenceFrame.getWorldFrame());
      FrameQuaternion orient = new FrameQuaternion(frame, quat4d);
      orient.changeFrame(ReferenceFrame.getWorldFrame());

      FramePose3D pose = new FramePose3D(point1, orient);

      return pose;
   }

   @Override
   public void onBehaviorExited()
   {
      publishTextToSpeech("Leaving Walk Through Door behavior");

   }

   private void lookDown()
   {
      AxisAngle orientationAxisAngle = new AxisAngle(0.0, 1.0, 0.0, 0.7);
      Quaternion headOrientation = new Quaternion();
      headOrientation.set(orientationAxisAngle);
      HeadTrajectoryMessage headTrajectoryMessage = HumanoidMessageTools.createHeadTrajectoryMessage(1.0,
                                                                                                     headOrientation,
                                                                                                     atlasPrimitiveActions.referenceFrames.getChestFrame(),
                                                                                                     atlasPrimitiveActions.referenceFrames.getChestFrame());
      headTrajectoryMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
      headTrajectoryPublisher.publish(headTrajectoryMessage);
   }

   private void lookUp()
   {
      AxisAngle orientationAxisAngle = new AxisAngle(0.0, 1.0, 0.0, 0);
      Quaternion headOrientation = new Quaternion();
      headOrientation.set(orientationAxisAngle);
      HeadTrajectoryMessage headTrajectoryMessage = HumanoidMessageTools.createHeadTrajectoryMessage(1.0,
                                                                                                     headOrientation,
                                                                                                     atlasPrimitiveActions.referenceFrames.getChestFrame(),
                                                                                                     atlasPrimitiveActions.referenceFrames.getChestFrame());
      headTrajectoryMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
      headTrajectoryPublisher.publish(headTrajectoryMessage);
   }

   private boolean isWalkingDone()
   {
      return walkToInteractableObjectBehavior.isDone();
   }

   private boolean hasWalkingSucceded()
   {
      return walkToInteractableObjectBehavior.succeded();
   }

   @Override
   public MessagerAPI getBehaviorAPI()
   {
      return MessengerAPI.create();
   }

   public static class MessengerAPI
   {
      private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
      private static final Category Root = apiFactory.createRootCategory("WalkThroughDoor");
      private static final CategoryTheme BEHAVIOR = apiFactory.createCategoryTheme("Behavior");

      public static final Topic<Boolean> Started = Root.child(BEHAVIOR).topic(apiFactory.createTypedTopicTheme("Started"));
      public static final Topic<String> State = Root.child(BEHAVIOR).topic(apiFactory.createTypedTopicTheme("State"));

      public static final MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }

}
