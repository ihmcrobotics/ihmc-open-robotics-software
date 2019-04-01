package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.UIPositionCheckerPacket;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.taskExecutor.ArmTrajectoryTask;
import us.ihmc.humanoidBehaviors.taskExecutor.HandDesiredConfigurationTask;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.frames.CommonReferenceFrameIds;
import us.ihmc.tools.taskExecutor.PipeLine;
import us.ihmc.yoVariables.variable.YoDouble;

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

   private final ResetRobotBehavior resetRobotBehavior;
   //   private final PassPacketBehavior passPacketBehavior;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final IHMCROS2Publisher<UIPositionCheckerPacket> uiPositionCheckerPacketpublisher;

   public GraspAndTurnValveBehavior(String robotName, YoDouble yoTime, Ros2Node ros2Node, AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super(robotName, ros2Node);
      this.atlasPrimitiveActions = atlasPrimitiveActions;

      resetRobotBehavior = new ResetRobotBehavior(robotName, ros2Node, yoTime);
      uiPositionCheckerPacketpublisher = createBehaviorOutputPublisher(UIPositionCheckerPacket.class);
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

      ArmTrajectoryMessage rightHandValveApproachMessage = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.RIGHT, 2, approachPointLocation);

      ArmTrajectoryTask moveHandToApproachPoint = new ArmTrajectoryTask(rightHandValveApproachMessage, atlasPrimitiveActions.rightArmTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            super.setBehaviorInput();
            publishTextToSpeech("Moving Hand To Approach Location");
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
            publishTextToSpeech("rotate Valve");
            FramePose3D point = offsetPointFromValveInWorldFrame(0.0, valveRadius + valveRadiusfinalOffset, distanceFromValve, 1.5708, 1.5708, -3.14159);

            GeometryTools.rotatePoseAboutAxis(valvePose, Axis.Z, degrees, point);

            uiPositionCheckerPacketpublisher.publish(MessageTools.createUIPositionCheckerPacket(point.getPosition()));

            HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(RobotSide.RIGHT, 2, point.getPosition(),
                                                                                                           point.getOrientation(),
                                                                                                           CommonReferenceFrameIds.CHEST_FRAME.getHashId());
            handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));

            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(handTrajectoryMessage);
         }
      };
      return moveHandAroundToValve;
   }

   private void moveHand(final double x, final double y, final double z, final double yaw, final double pitch, final double roll, final String description)
   {
      publishTextToSpeech(description);

      //      Vector3d orient = new Vector3d();
      //      referenceFrames.getHandFrame(RobotSide.RIGHT).getTransformToDesiredFrame(valvePose).getRotationEuler(orient);

      //      1.607778783110418,1.442441289823466,-3.1298946145335043`
      FramePose3D point = offsetPointFromValveInWorldFrame(x, y, z, yaw, pitch, roll);
      //      System.out.println("-orient.x,orient.y, orient.z " + (-orient.x) + "," + orient.y + "," + orient.z);

      uiPositionCheckerPacketpublisher.publish(MessageTools.createUIPositionCheckerPacket(point.getPosition()));

      HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(RobotSide.RIGHT, 2, point.getPosition(),
                                                                                                     point.getOrientation(),
                                                                                                     CommonReferenceFrameIds.CHEST_FRAME.getHashId());
      handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));

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

   private FramePose3D offsetPointFromValveInWorldFrame(double x, double y, double z, double yaw, double pitch, double roll)
   {
      FramePoint3D point1 = new FramePoint3D(valvePose, x, y, z);
      point1.changeFrame(ReferenceFrame.getWorldFrame());
      FrameQuaternion orient = new FrameQuaternion(valvePose, yaw, pitch, roll);
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