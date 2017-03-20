package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.communication.packets.UIPositionCheckerPacket;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.taskExecutor.ArmTrajectoryTask;
import us.ihmc.humanoidBehaviors.taskExecutor.HandDesiredConfigurationTask;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.taskExecutor.PipeLine;

public class GraspAndTurnValveBehavior extends AbstractBehavior
{
   private final PipeLine<AbstractBehavior> pipeLine = new PipeLine<>();

   private PoseReferenceFrame valvePose = null;
   private double valveRadius = 0;
   //   private double valveRadiusInitalOffset = 0.125;
   private double valveRadiusfinalOffset = -0.055;
   private double valveInitalForwardOffset = 0.125;
   private double valveFinalForwardOffset = 0.0225;

   private final double DEGREES_TO_ROTATE = 220;
   private final double ROTATION_SEGMENTS = 10;

   private final AtlasPrimitiveActions atlasPrimitiveActions;
   private final HumanoidReferenceFrames referenceFrames;

   private final ResetRobotBehavior resetRobotBehavior;
   //   private final PassPacketBehavior passPacketBehavior;


   public GraspAndTurnValveBehavior(DoubleYoVariable yoTime, HumanoidReferenceFrames referenceFrames, CommunicationBridge outgoingCommunicationBridge,
         AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super(outgoingCommunicationBridge);
      this.referenceFrames = referenceFrames;
      this.atlasPrimitiveActions = atlasPrimitiveActions;

      resetRobotBehavior = new ResetRobotBehavior(communicationBridge, yoTime);
      //      passPacketBehavior = new PassPacketBehavior(outgoingCommunicationBridge);

   }

   @Override
   public void onBehaviorEntered()
   {
      setupPipeline();
   }

   private void setupPipeline()
   {
      BehaviorAction resetRobot = new BehaviorAction(resetRobotBehavior);

      //CLOSE_HAND
      HandDesiredConfigurationTask closeHand = new HandDesiredConfigurationTask(RobotSide.RIGHT, HandConfiguration.CLOSE,
            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior);

      HandDesiredConfigurationTask openHand = new HandDesiredConfigurationTask(RobotSide.RIGHT, HandConfiguration.OPEN,
            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior);
      HandDesiredConfigurationTask openFingersOnly = new HandDesiredConfigurationTask(RobotSide.RIGHT, HandConfiguration.OPEN_FINGERS,
            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior);

      HandDesiredConfigurationTask closeThumb = new HandDesiredConfigurationTask(RobotSide.RIGHT, HandConfiguration.CLOSE_THUMB,
            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior);

      //    MOVE_HAND_TO_APPROACH_POINT, using joint angles to make sure wrist is in proper turn location

      double[] approachPointLocation = new double[] {0.42441454428847003, -0.5829781169010966, 1.8387098771297432, -2.35619, 0.11468460263836734,
            1.0402909950400858, 0.9434293109027067};

      ArmTrajectoryMessage rightHandValveApproachMessage = new ArmTrajectoryMessage(RobotSide.RIGHT, 2, approachPointLocation);

      ArmTrajectoryTask moveHandToApproachPoint = new ArmTrajectoryTask(rightHandValveApproachMessage, atlasPrimitiveActions.rightArmTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            super.setBehaviorInput();
            TextToSpeechPacket p1 = new TextToSpeechPacket("Moving Hand To Approach Location");
            sendPacket(p1);
         }
      };

      //      MOVE_HAND_ABOVE_AND_IN_FRONT_OF_VALVE,
      //      BehaviorAction moveHandToApproachPointViaHandTrajectory = moveHand(-0.05357945674961795, 0.04256176948547363, 0.15017291083938378, Math.toRadians(-86.487420629448),
      //            Math.toRadians(98.3359137072094), Math.toRadians(0.5464370649115582), "testPose");
      BehaviorAction moveHandInFrontOfValve = new BehaviorAction(atlasPrimitiveActions.rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            moveHand(0.0, valveRadius + valveRadiusfinalOffset, valveInitalForwardOffset, 1.5708, 1.5708, -3.14159, "Moving Hand In Front Of The Valve");
         }
      };

      BehaviorAction moveHandCloseToValve = new BehaviorAction(atlasPrimitiveActions.rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            moveHand(0.0, valveRadius + valveRadiusfinalOffset, valveFinalForwardOffset, 1.5708, 1.5708, -3.14159, "Aligning Hand With The Valve");
         }
      };

      BehaviorAction moveHandToValveGraspLocation = new BehaviorAction(atlasPrimitiveActions.rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            moveHand(0.0, valveRadius + valveRadiusfinalOffset, 0.0, 1.5708, 1.5708, -3.14159, "Moving Hand To Grasp Valve");
         }
      };

      //    CLOSE_HAND,
      pipeLine.submitSingleTaskStage(closeHand);

      //    MOVE_HAND_TO_APPROACH_POINT,
      pipeLine.submitSingleTaskStage(moveHandToApproachPoint);
      pipeLine.submitSingleTaskStage(openFingersOnly);
      pipeLine.submitSingleTaskStage(moveHandInFrontOfValve);
      //    MOVE_HAND_ABOVE_VALVE,
      pipeLine.submitSingleTaskStage(moveHandCloseToValve);

     

      //    MOVE_HAND_DOWN_TO_VALVE,
      pipeLine.submitSingleTaskStage(moveHandToValveGraspLocation);

      //    CLOSE_FINGERS,
      pipeLine.submitSingleTaskStage(closeHand);

      //    ROTATE,

      for (int i = 1; i <= ROTATION_SEGMENTS; i++)
      {
         pipeLine.submitSingleTaskStage(rotateAroundValve(Math.toRadians(-(DEGREES_TO_ROTATE / ROTATION_SEGMENTS) * i), 0.0));

      }

      //    OPEN_FINGERS_ONLY,
      pipeLine.submitSingleTaskStage(openFingersOnly);

      //    MOVE_HAND_AWAY_FROM_VALVE,

      pipeLine.submitSingleTaskStage(rotateAroundValve(Math.toRadians(-DEGREES_TO_ROTATE), valveInitalForwardOffset));

      pipeLine.submitSingleTaskStage(resetRobot);

   }

   private BehaviorAction rotateAroundValve(final double degrees, final double distanceFromValve)
   {
      BehaviorAction moveHandAroundToValve = new BehaviorAction(atlasPrimitiveActions.rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("rotate Valve");
            sendPacket(p1);
            FramePose point = offsetPointFromValveInWorldFrame(0.0, valveRadius + valveRadiusfinalOffset, distanceFromValve, 1.5708, 1.5708, -3.14159);

            point.rotatePoseAboutAxis(valvePose, Axis.Z, degrees);

            sendPacketToUI(new UIPositionCheckerPacket(point.getFramePointCopy().getPoint()));

            HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, 2, point.getFramePointCopy().getPoint(),
                  point.getFrameOrientationCopy().getQuaternion());

            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(handTrajectoryMessage);
         }
      };
      return moveHandAroundToValve;
   }

   private void moveHand(final double x, final double y, final double z, final double yaw, final double pitch, final double roll, final String description)
   {
      TextToSpeechPacket p1 = new TextToSpeechPacket(description);
      sendPacket(p1);

      //      Vector3d orient = new Vector3d();
      //      referenceFrames.getHandFrame(RobotSide.RIGHT).getTransformToDesiredFrame(valvePose).getRotationEuler(orient);

      //      1.607778783110418,1.442441289823466,-3.1298946145335043`
      FramePose point = offsetPointFromValveInWorldFrame(x, y, z, yaw, pitch, roll);
      //      System.out.println("-orient.x,orient.y, orient.z " + (-orient.x) + "," + orient.y + "," + orient.z);

      sendPacketToUI(new UIPositionCheckerPacket(point.getFramePointCopy().getPoint()));

      HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, 2, point.getFramePointCopy().getPoint(),
            point.getFrameOrientationCopy().getQuaternion());

      atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(handTrajectoryMessage);
   }

   public void setGrabLocation(PoseReferenceFrame valvePose, double valveRadius)
   {
      this.valvePose = valvePose;
      this.valveRadius = valveRadius;
   }

   @Override
   public void onBehaviorExited()
   {
      valvePose = null;
   }

   @Override
   public void doControl()
   {
      if (!isPaused())
         pipeLine.doControl();
   }

   @Override
   public boolean isDone()
   {
      return pipeLine.isDone();
   }

   private FramePose offsetPointFromValveInWorldFrame(double x, double y, double z, double yaw, double pitch, double roll)
   {
      FramePoint point1 = new FramePoint(valvePose, x, y, z);
      point1.changeFrame(ReferenceFrame.getWorldFrame());
      FrameOrientation orient = new FrameOrientation(valvePose, yaw, pitch, roll);
      orient.changeFrame(ReferenceFrame.getWorldFrame());

      FramePose pose = new FramePose(point1, orient);

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