package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3f;

import us.ihmc.communication.packets.TextToSpeechPacket;
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
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
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

      //      MOVE_HAND_ABOVE_VALVE,
      
      BehaviorAction moveHandAboveAndInFrontOfValve = new BehaviorAction(atlasPrimitiveActions.rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Moveing Hand Above And In Front Of The Valve");
            sendPacket(p1);
            
            Point3d point = offsetPointFromValveInChestFrame(0.0, valveRadius+valveRadiusInitalOffset, valveRadiusInitalForwardOffset);
            HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, 2, point, new Quat4d()); 
            
            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(handTrajectoryMessage);
         }
      };
      
      BehaviorAction moveHandAboveValve = new BehaviorAction(atlasPrimitiveActions.rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Moveing Hand Above And In Front Of The Valve");
            sendPacket(p1);
            
            Point3d point = offsetPointFromValveInChestFrame(0.0, valveRadius+valveRadiusInitalOffset, 0.0);
            HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, 2, point, new Quat4d()); 
            
            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(handTrajectoryMessage);
         }
      };

      //      MOVE_HAND_DOWN_TO_VALVE,

      BehaviorAction moveHandDownToValve = new BehaviorAction(atlasPrimitiveActions.rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Moving Hand Down To Valve");
            sendPacket(p1);
         }
      };

      //      ROTATE,
      //THIS Will Be Its Own Behavior
      //      MOVE_HAND_AWAY_FROM_VALVE,
      BehaviorAction moveHandAwayFromValve = new BehaviorAction(atlasPrimitiveActions.rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Moving Hand Away From Valve");
            sendPacket(p1);
         }
      };

      //    CLOSE_HAND,
      pipeLine.submitSingleTaskStage(closeHand);

      //    MOVE_HAND_TO_APPROACH_POINT,
      pipeLine.submitSingleTaskStage(moveHandAboveAndInFrontOfValve);
      pipeLine.submitSingleTaskStage(moveHandAboveValve);

      //    MOVE_HAND_ABOVE_VALVE,
      pipeLine.submitSingleTaskStage(moveHandAboveAndInFrontOfValve);
      //    OPEN_HAND,
      //    CLOSE_THUMB,
      //    MOVE_HAND_DOWN_TO_VALVE,
      //    CLOSE_FINGERS,
      //    ROTATE,
      //    OPEN_FINGERS_ONLY,
      //    MOVE_HAND_AWAY_FROM_VALVE,

//      pipeLine.submitSingleTaskStage(leftHandBeforeGrab);

   }

   public void setGrabLocation(PoseReferenceFrame valvePose , double valveRadius)
   {
      this.valvePose = valvePose;
      this.valveRadius = valveRadius;
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

   private Point3d offsetPointFromValveInChestFrame(double x, double y, double z)
   {
      FramePoint point1 = new FramePoint(valvePose, x, y, z);
      point1.changeFrame(referenceFrames.getChestFrame());
      return new Point3d(point1.getX(), point1.getY(), point1.getZ());
   }
}