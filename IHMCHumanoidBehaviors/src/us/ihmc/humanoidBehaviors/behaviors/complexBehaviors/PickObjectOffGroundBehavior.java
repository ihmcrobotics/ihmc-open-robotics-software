package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import javax.vecmath.Point3d;

import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.coactiveElements.PickUpBallBehaviorCoactiveElementBehaviorSide;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.taskExecutor.ArmTrajectoryTask;
import us.ihmc.humanoidBehaviors.taskExecutor.GoHomeTask;
import us.ihmc.humanoidBehaviors.taskExecutor.HandDesiredConfigurationTask;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage.BodyPart;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.taskExecutor.PipeLine;

public class PickObjectOffGroundBehavior extends AbstractBehavior
{

   private final PipeLine<AbstractBehavior> pipeLine = new PipeLine<>();


   private Point3d grabLocation = null;
   private double objectRadius = 0;
   private final AtlasPrimitiveActions atlasPrimitiveActions;

   public PickObjectOffGroundBehavior(DoubleYoVariable yoTime, PickUpBallBehaviorCoactiveElementBehaviorSide coactiveElement,
         HumanoidReferenceFrames referenceFrames, CommunicationBridge outgoingCommunicationBridge, AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super(outgoingCommunicationBridge);
      this.atlasPrimitiveActions = atlasPrimitiveActions;

      setupPipeline();
   }

   private void setupPipeline()
   {
      
      HandDesiredConfigurationTask openHand = new HandDesiredConfigurationTask(RobotSide.LEFT, HandConfiguration.OPEN, atlasPrimitiveActions.leftHandDesiredConfigurationBehavior);
      HandDesiredConfigurationTask closeHand = new HandDesiredConfigurationTask(RobotSide.LEFT, HandConfiguration.CLOSE, atlasPrimitiveActions.leftHandDesiredConfigurationBehavior);

      
      double[] leftHandAfterGrabLocation = new double[] {-0.799566492522621, -0.8850712601496326, 1.1978163314288173, 0.9978871050058826, -0.22593401111949774,
            -0.2153318563363089, -1.2957848304397805};

      ArmTrajectoryMessage leftHandAfterGrabMessage = new ArmTrajectoryMessage(RobotSide.LEFT, 2, leftHandAfterGrabLocation);

      ArmTrajectoryTask leftHandBeforeGrab = new ArmTrajectoryTask(leftHandAfterGrabMessage, atlasPrimitiveActions.leftArmTrajectoryBehavior);
      ArmTrajectoryTask leftHandAfterGrab = new ArmTrajectoryTask(leftHandAfterGrabMessage, atlasPrimitiveActions.leftArmTrajectoryBehavior);

      
   // GO TO INITIAL POICKUP LOCATION *******************************************
      BehaviorAction goToPickUpBallInitialLocationTask = new BehaviorAction(atlasPrimitiveActions.wholeBodyBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Picking Up The Ball");
            sendPacket(p1);
            FramePoint point = new FramePoint(ReferenceFrame.getWorldFrame(), grabLocation.getX(),
                  grabLocation.getY(),
                  grabLocation.getZ() + objectRadius + 0.25);
            atlasPrimitiveActions.wholeBodyBehavior.setSolutionQualityThreshold(2.01);
            atlasPrimitiveActions.wholeBodyBehavior.setTrajectoryTime(3);
            FrameOrientation tmpOr = new FrameOrientation(point.getReferenceFrame(), Math.toRadians(45), Math.toRadians(90), 0);
            atlasPrimitiveActions.wholeBodyBehavior.setDesiredHandPose(RobotSide.LEFT, point, tmpOr);

         }
      };

      //REACH FOR THE BALL *******************************************
      BehaviorAction pickUpBallTask = new BehaviorAction(atlasPrimitiveActions.wholeBodyBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            FramePoint point = new FramePoint(ReferenceFrame.getWorldFrame(), grabLocation.getX(),
                  grabLocation.getY(),
                  grabLocation.getZ() + objectRadius);
            atlasPrimitiveActions.wholeBodyBehavior.setSolutionQualityThreshold(2.01);
            atlasPrimitiveActions.wholeBodyBehavior.setTrajectoryTime(3);
            FrameOrientation tmpOr = new FrameOrientation(point.getReferenceFrame(), Math.toRadians(45), Math.toRadians(90), 0);
            atlasPrimitiveActions.wholeBodyBehavior.setDesiredHandPose(RobotSide.LEFT, point, tmpOr);

         }
      };

      GoHomeMessage goHomeChestMessage = new GoHomeMessage(BodyPart.CHEST, 2);
      GoHomeTask goHomeChestTask = new GoHomeTask(goHomeChestMessage, atlasPrimitiveActions.chestGoHomeBehavior);

      GoHomeMessage goHomepelvisMessage = new GoHomeMessage(BodyPart.PELVIS, 2);
      GoHomeTask goHomePelvisTask = new GoHomeTask(goHomepelvisMessage, atlasPrimitiveActions.pelvisGoHomeBehavior);

      pipeLine.submitSingleTaskStage(leftHandBeforeGrab);

      pipeLine.requestNewStage();

      pipeLine.submitTaskForPallelPipesStage(atlasPrimitiveActions.wholeBodyBehavior, goToPickUpBallInitialLocationTask);
      pipeLine.submitTaskForPallelPipesStage(atlasPrimitiveActions.wholeBodyBehavior, openHand);

      pipeLine.requestNewStage();

      pipeLine.submitSingleTaskStage(pickUpBallTask);
      pipeLine.submitSingleTaskStage(closeHand);

      pipeLine.submitSingleTaskStage(leftHandAfterGrab);
      //

      pipeLine.submitSingleTaskStage(goHomeChestTask);
      pipeLine.submitSingleTaskStage(goHomePelvisTask);

   }

   public void setGrabLocation(Point3d grabLocation, double objectRadius)
   {
      this.grabLocation = grabLocation;
      this.objectRadius = objectRadius;
   }

   @Override
   public void onBehaviorExited()
   {
      grabLocation = null;
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

   @Override
   public void onBehaviorEntered()
   {
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