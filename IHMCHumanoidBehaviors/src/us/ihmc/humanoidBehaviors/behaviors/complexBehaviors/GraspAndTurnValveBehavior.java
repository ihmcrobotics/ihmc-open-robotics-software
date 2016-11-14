package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3f;

import com.jme3.math.Quaternion;

import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.communication.packets.UIPositionCheckerPacket;
import us.ihmc.graphics3DAdapter.jme.util.JMEDataTypeUtils;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.coactiveElements.PickUpBallBehaviorCoactiveElementBehaviorSide;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.taskExecutor.ArmTrajectoryTask;
import us.ihmc.humanoidBehaviors.taskExecutor.GoHomeTask;
import us.ihmc.humanoidBehaviors.taskExecutor.HandDesiredConfigurationTask;
import us.ihmc.humanoidBehaviors.taskExecutor.HandTrajectoryTask;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage.BodyPart;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.taskExecutor.PipeLine;

public class GraspAndTurnValveBehavior extends AbstractBehavior
{

   /**
           CLOSE_HAND,
      MOVE_HAND_TO_APPROACH_POINT,
      MOVE_HAND_ABOVE_VALVE,
      OPEN_HAND,
      CLOSE_THUMB,
      MOVE_HAND_DOWN_TO_VALVE,
      CLOSE_FINGERS,
      ROTATE,
      OPEN_FINGERS_ONLY,
      MOVE_HAND_AWAY_FROM_VALVE,
      
    */

   private final PipeLine<AbstractBehavior> pipeLine = new PipeLine<>();

   private PoseReferenceFrame valvePose = null;
   private double valveRadius = 0;
   private double valveRadiusInitalOffset = 0.127;
   private double valveRadiusInitalForwardOffset = 0.2032;

   private final AtlasPrimitiveActions atlasPrimitiveActions;
   private final HumanoidReferenceFrames referenceFrames;

   public GraspAndTurnValveBehavior(DoubleYoVariable yoTime, HumanoidReferenceFrames referenceFrames, CommunicationBridge outgoingCommunicationBridge,
         AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super(outgoingCommunicationBridge);
      this.referenceFrames = referenceFrames;
      this.atlasPrimitiveActions = atlasPrimitiveActions;

      setupPipeline();
   }

   private void setupPipeline()
   {
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
      BehaviorAction moveHandAboveAndInFrontOfValve = moveHand(0.0, valveRadius + valveRadiusInitalOffset, valveRadiusInitalForwardOffset, Math.toRadians(-90),
            Math.toRadians(90), 0.0, "Moving Hand Above And In Front Of The Valve");
      //      MOVE_HAND_ABOVE_VALVE,

      BehaviorAction moveHandAboveValve = moveHand(0.0, valveRadius + valveRadiusInitalOffset, 0.0, Math.toRadians(-90), Math.toRadians(90), 0.0,
            "Moving Hand Above The Valve");

      //      MOVE_HAND_DOWN_TO_VALVE,

      BehaviorAction moveHandDownToValve = moveHand(0.0, valveRadius, 0.0, Math.toRadians(-90), Math.toRadians(90), 0.0, "Moving Hand Down To Valve");

      //      ROTATE,
      //      MOVE_HAND_AWAY_FROM_VALVE,
      BehaviorAction moveHandAwayFromValve = moveHand(0.0, -valveRadius - valveRadiusInitalOffset, 0.0, Math.toRadians(90), Math.toRadians(90), 0.0,
            "Moving Hand Away From Valve");

      //    CLOSE_HAND,
      pipeLine.submitSingleTaskStage(closeHand);

      //    MOVE_HAND_TO_APPROACH_POINT,
      pipeLine.submitSingleTaskStage(moveHandToApproachPoint);
//      pipeLine.submitSingleTaskStage(moveHandAboveAndInFrontOfValve);
      //    MOVE_HAND_ABOVE_VALVE,
      pipeLine.submitSingleTaskStage(moveHandAboveValve);

      //    OPEN_HAND,
      pipeLine.submitSingleTaskStage(openFingersOnly);

      //    MOVE_HAND_DOWN_TO_VALVE,
      pipeLine.submitSingleTaskStage(moveHandDownToValve);

      //    CLOSE_FINGERS,
      pipeLine.submitSingleTaskStage(closeHand);

      //    ROTATE,

      for (int i = 1; i < 10; i++)
      {
         pipeLine.submitSingleTaskStage(rotateAroundValve(Math.toRadians(-18 * i)));

      }

      //    OPEN_FINGERS_ONLY,
      pipeLine.submitSingleTaskStage(openFingersOnly);

      //    MOVE_HAND_AWAY_FROM_VALVE,

            pipeLine.submitSingleTaskStage(moveHandAwayFromValve);

   }

   private BehaviorAction rotateAroundValve(final double degrees)
   {
      BehaviorAction moveHandAroundToValve = new BehaviorAction(atlasPrimitiveActions.rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {

            FramePose point = offsetPointFromValveInWorldFrame(0.0, valveRadius, 0.0, 0, Math.toRadians(90), Math.toRadians(90));

            point.rotatePoseAboutAxis(valvePose, Axis.Z, degrees);

            sendPacketToUI(new UIPositionCheckerPacket(new Vector3f((float) point.getFramePointCopy().getPoint().getX(),
                  (float) point.getFramePointCopy().getPoint().getY(), (float) point.getFramePointCopy().getPoint().getZ())));

            HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, 1, point.getFramePointCopy().getPoint(),
                  point.getFrameOrientationCopy().getQuaternion());

            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(handTrajectoryMessage);
         }
      };
      return moveHandAroundToValve;
   }

   private BehaviorAction moveHand(final double x, final double y, final double z, final double yaw, final double pitch, final double roll,
         final String description)
   {
      BehaviorAction moveHand = new BehaviorAction(atlasPrimitiveActions.rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {

            TextToSpeechPacket p1 = new TextToSpeechPacket(description);
            sendPacket(p1);

            FramePose point = offsetPointFromValveInWorldFrame(x, y, z, yaw, pitch, roll);

            sendPacketToUI(new UIPositionCheckerPacket(new Vector3f((float) point.getX(), (float) point.getY(), (float) point.getZ())));
            
            HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, 1, point.getFramePointCopy().getPoint(),
                  point.getFrameOrientationCopy().getQuaternion());

            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(handTrajectoryMessage);
         }
      };
      return moveHand;
   }

   public void setGrabLocation(PoseReferenceFrame valvePose, double valveRadius)
   {
      this.valvePose = valvePose;
      this.valveRadius = valveRadius;
      System.out.println("valveRadius " + valveRadius);
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      super.doPostBehaviorCleanup();
      valvePose = null;
   }

   @Override
   public void doControl()
   {
      pipeLine.doControl();
   }

   @Override
   public boolean isDone()
   {
      // TODO Auto-generated method stub
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

}