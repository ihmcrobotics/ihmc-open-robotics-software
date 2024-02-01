package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.AutomaticManipulationAbortMessage;
import perception_msgs.msg.dds.DoorLocationPacket;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.UIPositionCheckerPacket;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.DoorOpenDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.OpenPushDoorBehavior.OpenDoorState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SleepBehavior;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.frames.CommonReferenceFrameIds;
import us.ihmc.yoVariables.variable.YoDouble;

public class OpenPushDoorBehavior extends StateMachineBehavior<OpenDoorState>
{

   enum OpenDoorState
   {
      START,
      MOVE_HANDs_TO_INITIAL_LOCATION,
      //MOVE_RIGHT_HAND_TO_INITIAL_LOCATION,
      TURN_ON_OPEN_DOOR_DETECTOR,
      TURN_DOOR_KNOB,
      PUSH_ON_DOOR,
      PUSH_OPEN_DOOR,
      PULL_BACK_HANDS,
      DONE,
      FAILED
   }

   private PoseReferenceFrame doorPoseFrame = null;

   private boolean succeeded;

   private final AtlasPrimitiveActions atlasPrimitiveActions;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private SleepBehavior sleepBehavior;
   private final IHMCROS2Publisher<UIPositionCheckerPacket> uiPositionCheckerPacketpublisher;
   protected final AtomicReference<DoorLocationPacket> doorLocationPacket = new AtomicReference<DoorLocationPacket>();
  // private final DoorOpenDetectorBehaviorService doorOpenDetectorBehaviorService;
   
   private long timeFirstDoorPushFinished = Long.MAX_VALUE;

   private final IHMCROS2Publisher<AutomaticManipulationAbortMessage> abortMessagePublisher;

   public OpenPushDoorBehavior(String robotName, String behaviorPrefix, YoDouble yoTime, ROS2Node ros2Node, AtlasPrimitiveActions atlasPrimitiveActions,
                               DoorOpenDetectorBehaviorService doorOpenDetectorBehaviorService, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(robotName, "OpenDoorBehavior", OpenDoorState.class, yoTime, ros2Node);
      this.atlasPrimitiveActions = atlasPrimitiveActions;
     // this.doorOpenDetectorBehaviorService = doorOpenDetectorBehaviorService;
      uiPositionCheckerPacketpublisher = createBehaviorOutputPublisher(UIPositionCheckerPacket.class);
      sleepBehavior = new SleepBehavior(robotName, ros2Node, yoTime);
      abortMessagePublisher = createPublisherForController(AutomaticManipulationAbortMessage.class);

      createSubscriber(DoorLocationPacket.class, PerceptionAPI.OBJECT_DETECTOR_TOOLBOX.withRobot(robotName).withOutput(), doorLocationPacket::set);

      setupStateMachine();

   }

   @Override
   public void onBehaviorEntered()
   {
      succeeded = false;
      doorLocationPacket.set(null);
      super.onBehaviorEntered();
   }

   public boolean succeeded()
   {
      return succeeded;
   }

   @Override
   protected OpenDoorState configureStateMachineAndReturnInitialKey(StateMachineFactory<OpenDoorState, BehaviorAction> factory)
   {

      BehaviorAction start = new BehaviorAction()
      {
         @Override
         public void onEntry()
         {
            super.onEntry();
            doorLocationPacket.set(null);
         }

         @Override
         protected void setBehaviorInput()
         {

         }

         @Override
         public boolean isDone()
         {
        	 return true;
//            //wait for the door to be located and a baseline set for open detection
//            if (doorLocationPacket.get() != null)
//            {
//               setGrabLocation(doorLocationPacket.get().getDoorTransformToWorld());
//            }
//            return doorLocationPacket.get() != null;
         }

      };

      BehaviorAction moveHandsToDoor = new BehaviorAction(atlasPrimitiveActions.leftHandTrajectoryBehavior,atlasPrimitiveActions.rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            setAutomaticArmAbort(false);

            atlasPrimitiveActions.leftHandTrajectoryBehavior.setInput(moveHand(0.298,
                                                                               -0.147,
                                                                               1.097,
                                                                               1.2554068994570775,
                                                                               0.03416782147174632,
                                                                               0.26586161890007015,
                                                                               RobotSide.LEFT,
                                                                               "Moving Left Hand To Door",
                                                                               4));
            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand(0.769,
                    -0.095,
                    1.032,
                    1.549469789243062,
                    0.08444685410187032,
                    0.037877956817564146,
                    RobotSide.RIGHT,
                    "Moving Right Hand Above Door Knob",
                    4));

         }
      };

      BehaviorAction moveRightHandToDoor = new BehaviorAction(atlasPrimitiveActions.rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            setAutomaticArmAbort(false);

            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand(0.769,
                                                                                -0.095,
                                                                                1.032,
                                                                                1.549469789243062,
                                                                                0.08444685410187032,
                                                                                0.037877956817564146,
                                                                                RobotSide.RIGHT,
                                                                                "Moving Right Hand Above Door Knob",
                                                                                4));

         }
      };

//      BehaviorAction setDoorDetectorStart = new BehaviorAction()
//      {
//         @Override
//         protected void setBehaviorInput()
//         {
//            publishTextToSpeech("Starting Door Open Detector Service");
//
//            doorOpenDetectorBehaviorService.reset();
//            doorOpenDetectorBehaviorService.run(true);
//         }
//
//         @Override
//         public boolean isDone()
//         {
//            //wait for the door to be located and a baseline set for open detection
//            if (doorOpenDetectorBehaviorService.doorDetected())
//            {
//               publishTextToSpeech("Door Open Detector Service has closed door position saved");
//
//            }
//            return doorOpenDetectorBehaviorService.doorDetected();
//
//         }
//
//      };

      BehaviorAction moveRightHandToDoorKnob = new BehaviorAction(atlasPrimitiveActions.rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            setAutomaticArmAbort(true);

            //            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand(0.780, -0.0635, 0.879, 1.551252338779563, 0.048351007951384285,
            //                                                                                0.007252343575301105, RobotSide.RIGHT, "Moving Hand To Door Knob", 2));

            //atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand(0.8, -0.1, 0.879, 1.551252338779563, 0.048351007951384285,0.007252343575301105, RobotSide.RIGHT, "Moving Hand To Door Knob", 2));

            //      RIGHT hand in MultiClickdoor_0_objID1219 ( 0.769, -0.096,  0.932 ) orientation 1.5511648101378044, 0.08462087065219358, 0.03818089607481523

            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand(0.769,
                                                                                -0.096,
                                                                                0.932,
                                                                                1.5511648101378044,
                                                                                0.08462087065219358,
                                                                                0.03818089607481523,
                                                                                RobotSide.RIGHT,
                                                                                "Moving Hand To Door Knob",
                                                                                4));

         }
      };
      BehaviorAction pushDoorALittle = new BehaviorAction(atlasPrimitiveActions.rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            //atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand(0.75, -0.00, 0.879, 1.551252338779563, 0.048351007951384285,0.007252343575301105, RobotSide.RIGHT, "Push Door A Little", 1));
            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand(0.750,
                                                                                0.01,
                                                                                0.896,
                                                                                1.5911238903674156,
                                                                                0.038548649273740986,
                                                                                -7.31590778193919E-4,
                                                                                RobotSide.RIGHT,
                                                                                "Push Door A Little",
                                                                                2));
            //RIGHT hand in MultiClickdoor_0_objID197 ( 0.750, -0.049,  0.896 ) orientation 1.5911238903674156, 0.038548649273740986, -7.31590778193919E-4

         }
         
         @Override
         public void onEntry()
         {
            super.onEntry();
            timeFirstDoorPushFinished = Long.MAX_VALUE;
         }
         @Override
         public boolean isDone()
         {
            if(super.isDone()&&timeFirstDoorPushFinished==Long.MAX_VALUE)
               timeFirstDoorPushFinished = System.currentTimeMillis();
            return super.isDone();
         }

      };
      BehaviorAction pushDoorOpen = new BehaviorAction(atlasPrimitiveActions.leftHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            //otherwise the robot stops the arm motion because it is to fast
            setAutomaticArmAbort(false);

            atlasPrimitiveActions.leftHandTrajectoryBehavior.setInput(moveHand(0.317,  0.300,  1.264,
                                                                               1.472225053162252, 0.03597891325029729, 1.5545423993328358,
                                                                               RobotSide.LEFT,
                                                                               "Pushing Door",
                                                                               3));
         }

      };

      BehaviorAction pullHandsBack = new BehaviorAction(atlasPrimitiveActions.leftHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            setAutomaticArmAbort(true);

            atlasPrimitiveActions.leftHandTrajectoryBehavior.setInput(moveHand(0.350, -0.018,  0.882 ,
                                                                               1.4668096379070716, 0.03925846523623192, 1.6360375489656063,
                                                                               RobotSide.LEFT,
                                                                               "Pulling Left Hand Back",
                                                                               1));

         }
      };

      BehaviorAction done = new BehaviorAction()
      {
         @Override
         protected void setBehaviorInput()
         {
            succeeded = true;
            publishTextToSpeech("DOOR OPENING COMPLETE");
         }
      };

      BehaviorAction failed = new BehaviorAction(atlasPrimitiveActions.leftHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            succeeded = false;
            atlasPrimitiveActions.leftHandTrajectoryBehavior.setInput(moveHand(0.274,
                                                                               -0.208,
                                                                               0.798,
                                                                               1.2609443582725661,
                                                                               0.02096196100421688,
                                                                               0.27326972080173334,
                                                                               RobotSide.LEFT,
                                                                               "Pulling Left Hand Back",
                                                                               5));
            publishTextToSpeech("DOOR OPENING FAILED");
         }
      };

      factory.addStateAndDoneTransition(OpenDoorState.START, start, OpenDoorState.MOVE_HANDs_TO_INITIAL_LOCATION);
      factory.addStateAndDoneTransition(OpenDoorState.MOVE_HANDs_TO_INITIAL_LOCATION,
                                        moveHandsToDoor,
                                        OpenDoorState.TURN_DOOR_KNOB);
      //factory.addStateAndDoneTransition(OpenDoorState.MOVE_RIGHT_HAND_TO_INITIAL_LOCATION, moveRightHandToDoor, OpenDoorState.TURN_DOOR_KNOB);
     // factory.addStateAndDoneTransition(OpenDoorState.TURN_ON_OPEN_DOOR_DETECTOR, setDoorDetectorStart, OpenDoorState.TURN_DOOR_KNOB);

      factory.addStateAndDoneTransition(OpenDoorState.TURN_DOOR_KNOB, moveRightHandToDoorKnob, OpenDoorState.PUSH_ON_DOOR);



      factory.addState(OpenDoorState.PUSH_ON_DOOR, pushDoorALittle);
      factory.addState(OpenDoorState.PUSH_OPEN_DOOR, pushDoorOpen);
      factory.addState(OpenDoorState.PULL_BACK_HANDS, pullHandsBack);
      factory.addState(OpenDoorState.DONE, done);
      factory.addState(OpenDoorState.FAILED, failed);

     // factory.addTransition(OpenDoorState.PUSH_ON_DOOR, OpenDoorState.FAILED, t -> pushDoorALittle.isDone() &&doorOpenDetectorBehaviorService.getLastupdateTime()>=timeFirstDoorPushFinished && !doorOpenDetectorBehaviorService.isDoorOpen());
      factory.addTransition(OpenDoorState.PUSH_ON_DOOR, OpenDoorState.PUSH_OPEN_DOOR, t -> pushDoorALittle.isDone());//doorOpenDetectorBehaviorService.isDoorOpen());

      // factory.addTransition(OpenDoorState.PUSH_OPEN_DOOR, OpenDoorState.FAILED, t -> !doorOpenDetectorBehaviorService.isDoorOpen());
      factory.addTransition(OpenDoorState.PUSH_OPEN_DOOR, OpenDoorState.PULL_BACK_HANDS, t -> pushDoorOpen.isDone());// && doorOpenDetectorBehaviorService.isDoorOpen());

      //factory.addTransition(OpenDoorState.PULL_BACK_HANDS, OpenDoorState.FAILED, t -> pullHandsBack.isDone() && !doorOpenDetectorBehaviorService.isDoorOpen());
      factory.addTransition(OpenDoorState.PULL_BACK_HANDS, OpenDoorState.DONE, t -> pullHandsBack.isDone());// && doorOpenDetectorBehaviorService.isDoorOpen());

      return OpenDoorState.START;

   }

   /*
    * @Override public boolean isDone() {
    * //System.out.println("done check "+super.isDone()+" "+getStateMachine().
    * getCurrentBehaviorKey()+" " +getStateMachine().getCurrentAction().isDone()); return
    * super.isDone(); }
    */

   private HandTrajectoryMessage moveHand(final double x, final double y, final double z, final double yaw, final double pitch, final double roll,
                                          final RobotSide side, final String description, double trajectoryTime)
   {
      publishTextToSpeech(description);

      FramePose3D point = offsetPointFromDoorInWorldFrame(x, y, z, yaw, pitch, roll);

      uiPositionCheckerPacketpublisher.publish(MessageTools.createUIPositionCheckerPacket(point.getPosition()));

      HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(side,
                                                                                                     trajectoryTime,
                                                                                                     point.getPosition(),
                                                                                                     point.getOrientation(),
                                                                                                     CommonReferenceFrameIds.CHEST_FRAME.getHashId());
      handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));

      return handTrajectoryMessage;
   }

   private void setAutomaticArmAbort(boolean enableAbort)
   {
      AutomaticManipulationAbortMessage automaticManipulationAbortPacket = HumanoidMessageTools.createAutomaticManipulationAbortMessage(enableAbort);
      abortMessagePublisher.publish(automaticManipulationAbortPacket);
   }

   public void setGrabLocation(Pose3D doorPose3D)
   {
      publishTextToSpeech("grab location set " + doorPose3D);
      PoseReferenceFrame doorPose = new PoseReferenceFrame("OpenDoorReferenceFrame", ReferenceFrame.getWorldFrame());
      doorPose.setPoseAndUpdate(new Pose3D(doorPose3D));
      this.doorPoseFrame = doorPose;
   }

   @Override
   public void doControl()
   {

      super.doControl();
   }

   @Override
   public void onBehaviorExited()
   {
      doorPoseFrame = null;

   }

   private FramePose3D offsetPointFromDoorInWorldFrame(double x, double y, double z, double yaw, double pitch, double roll)
   {
      System.out.println("doorPoseFrame " + doorPoseFrame);
      FramePoint3D point1 = new FramePoint3D(doorPoseFrame, x, y, z);
      point1.changeFrame(ReferenceFrame.getWorldFrame());
      FrameQuaternion orient = new FrameQuaternion(doorPoseFrame, yaw, pitch, roll);
      orient.changeFrame(ReferenceFrame.getWorldFrame());

      FramePose3D pose = new FramePose3D(point1, orient);

      return pose;
   }

   @Override
   public void onBehaviorAborted()
   {
   }

   @Override
   public void onBehaviorPaused()
   {
   }

   @Override
   public void onBehaviorResumed()
   {
   }

}