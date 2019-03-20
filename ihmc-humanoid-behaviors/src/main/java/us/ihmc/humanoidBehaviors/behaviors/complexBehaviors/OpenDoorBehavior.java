package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.UIPositionCheckerPacket;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.Pose3D;
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

public class OpenDoorBehavior extends AbstractBehavior
{
   private final PipeLine<AbstractBehavior> pipeLine = new PipeLine<>();

   private PoseReferenceFrame doorPoseFrame = null;


   private final AtlasPrimitiveActions atlasPrimitiveActions;

   private final ResetRobotBehavior resetRobotBehavior;
   //   private final PassPacketBehavior passPacketBehavior;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final IHMCROS2Publisher<UIPositionCheckerPacket> uiPositionCheckerPacketpublisher;

   public OpenDoorBehavior(String robotName, YoDouble yoTime, Ros2Node ros2Node, AtlasPrimitiveActions atlasPrimitiveActions)
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

      BehaviorAction moveHandsToDoor = new BehaviorAction(atlasPrimitiveActions.leftHandTrajectoryBehavior,atlasPrimitiveActions.rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
//            moveHand( 0.797,-0.083,0.912,  1.521 , 0, -1.641 , "Moving Hand Above Door Knob3");
            atlasPrimitiveActions.leftHandTrajectoryBehavior.setInput(moveHand( 0.335, -0.087,  1.124,1.2478615483800362, -0.12618033550878305, 0.3133655061333552,RobotSide.LEFT,"Moving Left Hand To Door"));
            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand(0.795, -0.06,  1.032 , 1.7318790859631, 0.9163508562370669, -0.2253954188985998,RobotSide.RIGHT, "Moving Right Hand Above Door Knob"));

         }
      };

      BehaviorAction moveRightHandToDoorKnob = new BehaviorAction(atlasPrimitiveActions.rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand(0.770, -0.092,  0.882,  1.7318790859631, 0.9163508562370669, -0.2253954188985998 , RobotSide.RIGHT,"Moving Hand To Door Knob"));
         }
      };
      BehaviorAction pushDoor = new BehaviorAction(atlasPrimitiveActions.leftHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            atlasPrimitiveActions.leftHandTrajectoryBehavior.setInput(moveHand(0.400,  0.096,  0.8,  1.244653857913857, -0.12493851224047543, 0.34615057210000433 , RobotSide.LEFT,"Pushing Door"));
//            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand(0.777, -0.032,  0.882, 1.7318790859631, 0.9163508562370669, -0.2253954188985998 , RobotSide.RIGHT,"Moving Hand To Door Knob"));

         }
      };

//      BehaviorAction moveHandCloseToValve = new BehaviorAction(atlasPrimitiveActions.rightHandTrajectoryBehavior)
//      {
//         @Override
//         protected void setBehaviorInput()
//         {
//            moveHand(0.0, valveRadius + valveRadiusfinalOffset, valveFinalForwardOffset, 1.5708, 1.5708, -3.14159, "Aligning Hand With The Valve");
//         }
//      };
//
//      BehaviorAction moveHandToValveGraspLocation = new BehaviorAction(atlasPrimitiveActions.rightHandTrajectoryBehavior)
//      {
//         @Override
//         protected void setBehaviorInput()
//         {
//            moveHand(0.0, valveRadius + valveRadiusfinalOffset, 0.0, 1.5708, 1.5708, -3.14159, "Moving Hand To Grasp Valve");
//         }
//      };

      //    CLOSE_HAND,
     // pipeLine.submitSingleTaskStage(closeHand);

      //    MOVE_HAND_TO_APPROACH_POINT,
      //pipeLine.submitSingleTaskStage(moveHandToApproachPoint);
     // pipeLine.submitSingleTaskStage(openFingersOnly);
      pipeLine.clearAll();
      pipeLine.submitSingleTaskStage(moveHandsToDoor);
      pipeLine.submitSingleTaskStage(moveRightHandToDoorKnob);
      pipeLine.submitSingleTaskStage(pushDoor);


      //pipeLine.submitSingleTaskStage(moveRightHandAboveDoorKnob);

      //    MOVE_HAND_ABOVE_VALVE,
      //pipeLine.submitSingleTaskStage(moveHandCloseToValve);

      //    MOVE_HAND_DOWN_TO_VALVE,
     // pipeLine.submitSingleTaskStage(moveHandToValveGraspLocation);

      //    CLOSE_FINGERS,
      //pipeLine.submitSingleTaskStage(closeHand);

      //    ROTATE,

     // for (int i = 1; i <= ROTATION_SEGMENTS; i++)
      //{
     //    pipeLine.submitSingleTaskStage(rotateAroundValve(Math.toRadians(-(DEGREES_TO_ROTATE / ROTATION_SEGMENTS) * i), 0.0));

     // }

      //    OPEN_FINGERS_ONLY,
     // pipeLine.submitSingleTaskStage(openFingersOnly);

      //    MOVE_HAND_AWAY_FROM_VALVE,

     // pipeLine.submitSingleTaskStage(rotateAroundValve(Math.toRadians(-DEGREES_TO_ROTATE), valveInitalForwardOffset));

      //pipeLine.submitSingleTaskStage(resetRobot);

   }

//   private BehaviorAction rotateAroundValve(final double degrees, final double distanceFromValve)
//   {
//      BehaviorAction moveHandAroundToValve = new BehaviorAction(atlasPrimitiveActions.rightHandTrajectoryBehavior)
//      {
//
//         @Override
//         protected void setBehaviorInput()
//         {
//            publishTextToSpeack("rotate Valve");
//            FramePose3D point = offsetPointFromValveInWorldFrame(0.0, valveRadius + valveRadiusfinalOffset, distanceFromValve, 1.5708, 1.5708, -3.14159);
//
//            GeometryTools.rotatePoseAboutAxis(valvePose, Axis.Z, degrees, point);
//
//            uiPositionCheckerPacketpublisher.publish(MessageTools.createUIPositionCheckerPacket(point.getPosition()));
//
//            HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(RobotSide.RIGHT, 2, point.getPosition(),
//                                                                                                           point.getOrientation(),
//                                                                                                           CommonReferenceFrameIds.CHEST_FRAME.getHashId());
//            handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));
//
//            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(handTrajectoryMessage);
//         }
//      };
//      return moveHandAroundToValve;
//   }

   private HandTrajectoryMessage moveHand(final double x, final double y, final double z, final double yaw, final double pitch, final double roll,final RobotSide side, final String description)
   {
      publishTextToSpeack(description);

      //      Vector3d orient = new Vector3d();
      //      referenceFrames.getHandFrame(RobotSide.RIGHT).getTransformToDesiredFrame(valvePose).getRotationEuler(orient);

      //      1.607778783110418,1.442441289823466,-3.1298946145335043`
      FramePose3D point = offsetPointFromDoorInWorldFrame(x, y, z, yaw, pitch, roll);
      //      System.out.println("-orient.x,orient.y, orient.z " + (-orient.x) + "," + orient.y + "," + orient.z);

      uiPositionCheckerPacketpublisher.publish(MessageTools.createUIPositionCheckerPacket(point.getPosition()));

      HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(side, 2, point.getPosition(),
                                                                                                     point.getOrientation(),
                                                                                                     CommonReferenceFrameIds.CHEST_FRAME.getHashId());
      handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));

     return handTrajectoryMessage;
   }

   public void setGrabLocation(Pose3D doorPose3D)
   {
      
      PoseReferenceFrame doorPose = new PoseReferenceFrame("OpenDoorReferenceFrame", ReferenceFrame.getWorldFrame());
      doorPose.setPoseAndUpdate(new Pose3D(doorPose3D));
      this.doorPoseFrame = doorPose;
   }

   @Override
   public void onBehaviorExited()
   {
      doorPoseFrame = null;
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

   private FramePose3D offsetPointFromDoorInWorldFrame(double x, double y, double z, double yaw, double pitch, double roll)
   {
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