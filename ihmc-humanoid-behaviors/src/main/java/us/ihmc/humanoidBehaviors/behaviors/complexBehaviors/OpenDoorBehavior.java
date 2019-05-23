package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.AutomaticManipulationAbortMessage;
import controller_msgs.msg.dds.DoorLocationPacket;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.HeadTrajectoryMessage;
import controller_msgs.msg.dds.UIPositionCheckerPacket;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.DoorOpenDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.OpenDoorBehavior.OpenDoorState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SleepBehavior;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.frames.CommonReferenceFrameIds;
import us.ihmc.yoVariables.variable.YoDouble;

public class OpenDoorBehavior extends StateMachineBehavior<OpenDoorState>
{

   enum OpenDoorState
   {
      START, MOVE_HANDS_TO_INITIAL_LOCATION,TURN_ON_OPEN_DOOR_DETECTOR, TURN_DOOR_KNOB, PUSH_ON_DOOR, PUSH_OPEN_DOOR, PULL_BACK_HANDS, DONE, FAILED
   }

   private PoseReferenceFrame doorPoseFrame = null;

   private boolean succeeded;

   private final AtlasPrimitiveActions atlasPrimitiveActions;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private SleepBehavior sleepBehavior;
   private final IHMCROS2Publisher<UIPositionCheckerPacket> uiPositionCheckerPacketpublisher;
   protected final AtomicReference<DoorLocationPacket> doorLocationReference = new AtomicReference<DoorLocationPacket>();
   private final DoorOpenDetectorBehaviorService doorOpenDetectorBehaviorService;

   private final IHMCROS2Publisher<AutomaticManipulationAbortMessage> abortMessagePublisher;

   public OpenDoorBehavior(String robotName, String behaviorPrefix, YoDouble yoTime, Ros2Node ros2Node, AtlasPrimitiveActions atlasPrimitiveActions,
                           DoorOpenDetectorBehaviorService doorOpenDetectorBehaviorService, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(robotName, "OpenDoorBehavior", OpenDoorState.class, yoTime, ros2Node);
      this.atlasPrimitiveActions = atlasPrimitiveActions;
      this.doorOpenDetectorBehaviorService = doorOpenDetectorBehaviorService;
      uiPositionCheckerPacketpublisher = createBehaviorOutputPublisher(UIPositionCheckerPacket.class);
      sleepBehavior = new SleepBehavior(robotName, ros2Node, yoTime);
      abortMessagePublisher = createPublisherForController(AutomaticManipulationAbortMessage.class);

      createBehaviorInputSubscriber(DoorLocationPacket.class, doorLocationReference::set);

      setupStateMachine();

   }

   @Override
   public void onBehaviorEntered()
   {
      succeeded = false;
      doorLocationReference.set(null);
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
         protected void setBehaviorInput()
         {

         }

         @Override
         public boolean isDone()
         {
            //wait for the door to be located and a baseline set for open detection
            return doorLocationReference.get() != null;
         }

      };
      BehaviorAction moveHandsToDoor = new BehaviorAction(atlasPrimitiveActions.rightHandTrajectoryBehavior, atlasPrimitiveActions.leftHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            setAutomaticArmAbort(false);

            //pre speedup values for distance from door -0.102
            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand(0.833, -0.0635, 1.079, 1.551252338779563, 0.048351007951384285,

                                                                                0.007252343575301105, RobotSide.RIGHT, "Moving Right Hand Above Door Knob",3));
            atlasPrimitiveActions.leftHandTrajectoryBehavior.setInput(moveHand(0.298, -0.147, 1.097, 1.2554068994570775, 0.03416782147174632,


         }
      };
      
      BehaviorAction setDoorDetectorStart = new BehaviorAction()
      {
         @Override
         protected void setBehaviorInput()
         {
            publishTextToSpeech("Starting Door Open Detector Service");

            doorOpenDetectorBehaviorService.reset();
            doorOpenDetectorBehaviorService.run(true);
         }

         @Override
         public boolean isDone()
         {
            //wait for the door to be located and a baseline set for open detection
            if(doorOpenDetectorBehaviorService.doorDetected())
            {
               publishTextToSpeech("Door Open Detector Service has closed door position saved");

            }
            return doorOpenDetectorBehaviorService.doorDetected();
         }

      };

      BehaviorAction moveRightHandToDoorKnob = new BehaviorAction(atlasPrimitiveActions.rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            setAutomaticArmAbort(true);

            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand(0.780, -0.0635, 0.879, 1.551252338779563, 0.048351007951384285,
                                                                                0.007252343575301105, RobotSide.RIGHT, "Moving Hand To Door Knob", 2));
         }
      };
      BehaviorAction pushDoorALittle = new BehaviorAction(atlasPrimitiveActions.rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand(0.780, -0.00, 0.879, 1.551252338779563, 0.048351007951384285,
                                                                                0.007252343575301105, RobotSide.RIGHT, "Push Door A Little", 1));

         }
      };
      BehaviorAction pushDoorOpen = new BehaviorAction(atlasPrimitiveActions.leftHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            //otherwise the robot stops the arm motion because it is to fast
            setAutomaticArmAbort(false);
            atlasPrimitiveActions.leftHandTrajectoryBehavior.setInput(moveHand(0.455, 0.218, 1.154, 1.7318790859631, 0.9163508562370669, -0.2253954188985998,
                                                                               RobotSide.LEFT, "Pushing Door", 1));
         }

      };

      BehaviorAction pullHandsBack = new BehaviorAction(atlasPrimitiveActions.leftHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            setAutomaticArmAbort(true);
            atlasPrimitiveActions.leftHandTrajectoryBehavior.setInput(moveHand(0.274, -0.208, 0.798, 1.2609443582725661, 0.02096196100421688,
                                                                               0.27326972080173334, RobotSide.LEFT, "Pulling Left Hand Back", 1));

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
            atlasPrimitiveActions.leftHandTrajectoryBehavior.setInput(moveHand(0.274, -0.208, 0.798, 1.2609443582725661, 0.02096196100421688,
                                                                               0.27326972080173334, RobotSide.LEFT, "Pulling Left Hand Back", 5));
            publishTextToSpeech("DOOR OPENING FAILED");
         }
      };

      factory.addStateAndDoneTransition(OpenDoorState.START, start, OpenDoorState.MOVE_HANDS_TO_INITIAL_LOCATION);
      factory.addStateAndDoneTransition(OpenDoorState.MOVE_HANDS_TO_INITIAL_LOCATION, moveHandsToDoor, OpenDoorState.TURN_ON_OPEN_DOOR_DETECTOR);
      factory.addStateAndDoneTransition(OpenDoorState.TURN_ON_OPEN_DOOR_DETECTOR, setDoorDetectorStart, OpenDoorState.TURN_DOOR_KNOB);

      factory.addStateAndDoneTransition(OpenDoorState.TURN_DOOR_KNOB, moveRightHandToDoorKnob, OpenDoorState.PUSH_ON_DOOR);
      factory.addState(OpenDoorState.PUSH_ON_DOOR, pushDoorALittle);
      factory.addState(OpenDoorState.PUSH_OPEN_DOOR, pushDoorOpen);
      factory.addState(OpenDoorState.PULL_BACK_HANDS, pullHandsBack);
      factory.addState(OpenDoorState.DONE, done);
      factory.addState(OpenDoorState.FAILED, failed);

      factory.addTransition(OpenDoorState.PUSH_ON_DOOR, OpenDoorState.FAILED, t -> pushDoorALittle.isDone() && !doorOpenDetectorBehaviorService.isDoorOpen());
      factory.addTransition(OpenDoorState.PUSH_ON_DOOR, OpenDoorState.PUSH_OPEN_DOOR, t -> doorOpenDetectorBehaviorService.isDoorOpen());

      
      //removing door open checks durring fast motions for now.
     // factory.addTransition(OpenDoorState.PUSH_OPEN_DOOR, OpenDoorState.FAILED, t -> !doorOpenDetectorBehaviorService.isDoorOpen());
      factory.addTransition(OpenDoorState.PUSH_OPEN_DOOR, OpenDoorState.PULL_BACK_HANDS, t -> pushDoorOpen.isDone());// && doorOpenDetectorBehaviorService.isDoorOpen());

      factory.addTransition(OpenDoorState.PULL_BACK_HANDS, OpenDoorState.FAILED, t -> pullHandsBack.isDone() && !doorOpenDetectorBehaviorService.isDoorOpen());
      factory.addTransition(OpenDoorState.PULL_BACK_HANDS, OpenDoorState.DONE, t -> pullHandsBack.isDone() && doorOpenDetectorBehaviorService.isDoorOpen());

      return OpenDoorState.START;

   }

   /*
    * @Override public boolean isDone() {
    * //System.out.println("done check "+super.isDone()+" "+getStateMachine().
    * getCurrentBehaviorKey()+" "
    * +getStateMachine().getCurrentAction().isDone()); return super.isDone(); }
    */

   private HandTrajectoryMessage moveHand(final double x, final double y, final double z, final double yaw, final double pitch, final double roll,
                                          final RobotSide side, final String description, double trajectoryTime)
   {
      publishTextToSpeech(description);

      FramePose3D point = offsetPointFromDoorInWorldFrame(x, y, z, yaw, pitch, roll);

      uiPositionCheckerPacketpublisher.publish(MessageTools.createUIPositionCheckerPacket(point.getPosition()));

      HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(side, trajectoryTime, point.getPosition(),
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