package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.GoHomeMessage;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.coactiveElements.PickUpBallBehaviorCoactiveElementBehaviorSide;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.taskExecutor.ArmTrajectoryTask;
import us.ihmc.humanoidBehaviors.taskExecutor.GoHomeTask;
import us.ihmc.humanoidBehaviors.taskExecutor.HandDesiredConfigurationTask;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.walking.HumanoidBodyPart;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.taskExecutor.PipeLine;
import us.ihmc.yoVariables.variable.YoDouble;

public class PickObjectOffGroundBehavior extends AbstractBehavior
{

   private final PipeLine<AbstractBehavior> pipeLine = new PipeLine<>();

   private Point3D grabLocation = null;
   private double objectRadius = 0;
   private final AtlasPrimitiveActions atlasPrimitiveActions;

   public PickObjectOffGroundBehavior(String robotName, YoDouble yoTime, PickUpBallBehaviorCoactiveElementBehaviorSide coactiveElement,
                                      HumanoidReferenceFrames referenceFrames, Ros2Node ros2Node, AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super(robotName, ros2Node);
      this.atlasPrimitiveActions = atlasPrimitiveActions;

      setupPipeline();
   }

   private void setupPipeline()
   {

      HandDesiredConfigurationTask openHand = new HandDesiredConfigurationTask(RobotSide.LEFT, HandConfiguration.OPEN,
                                                                               atlasPrimitiveActions.leftHandDesiredConfigurationBehavior);
      HandDesiredConfigurationTask closeHand = new HandDesiredConfigurationTask(RobotSide.LEFT, HandConfiguration.CLOSE,
                                                                                atlasPrimitiveActions.leftHandDesiredConfigurationBehavior);

      double[] leftHandAfterGrabLocation = new double[] {-0.799566492522621, -0.8850712601496326, 1.1978163314288173, 0.9978871050058826, -0.22593401111949774,
            -0.2153318563363089, -1.2957848304397805};

      ArmTrajectoryMessage leftHandAfterGrabMessage = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.LEFT, 2, leftHandAfterGrabLocation);

      ArmTrajectoryTask leftHandBeforeGrab = new ArmTrajectoryTask(leftHandAfterGrabMessage, atlasPrimitiveActions.leftArmTrajectoryBehavior);
      ArmTrajectoryTask leftHandAfterGrab = new ArmTrajectoryTask(leftHandAfterGrabMessage, atlasPrimitiveActions.leftArmTrajectoryBehavior);

      // GO TO INITIAL POICKUP LOCATION *******************************************
      BehaviorAction goToPickUpBallInitialLocationTask = new BehaviorAction(atlasPrimitiveActions.wholeBodyBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            publishTextToSpeech("Picking Up The Ball");
            FramePoint3D point = new FramePoint3D(ReferenceFrame.getWorldFrame(), grabLocation.getX(), grabLocation.getY(),
                                                  grabLocation.getZ() + objectRadius + 0.25);
            atlasPrimitiveActions.wholeBodyBehavior.setSolutionQualityThreshold(2.01);
            atlasPrimitiveActions.wholeBodyBehavior.setTrajectoryTime(3);
            FrameQuaternion tmpOr = new FrameQuaternion(point.getReferenceFrame(), Math.toRadians(45), Math.toRadians(90), 0);
            atlasPrimitiveActions.wholeBodyBehavior.setDesiredHandPose(RobotSide.LEFT, point, tmpOr);

         }
      };

      //REACH FOR THE BALL *******************************************
      BehaviorAction pickUpBallTask = new BehaviorAction(atlasPrimitiveActions.wholeBodyBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            FramePoint3D point = new FramePoint3D(ReferenceFrame.getWorldFrame(), grabLocation.getX(), grabLocation.getY(), grabLocation.getZ() + objectRadius);
            atlasPrimitiveActions.wholeBodyBehavior.setSolutionQualityThreshold(2.01);
            atlasPrimitiveActions.wholeBodyBehavior.setTrajectoryTime(3);
            FrameQuaternion tmpOr = new FrameQuaternion(point.getReferenceFrame(), Math.toRadians(45), Math.toRadians(90), 0);
            atlasPrimitiveActions.wholeBodyBehavior.setDesiredHandPose(RobotSide.LEFT, point, tmpOr);

         }
      };

      GoHomeMessage goHomeChestMessage = HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.CHEST, 2);
      GoHomeTask goHomeChestTask = new GoHomeTask(goHomeChestMessage, atlasPrimitiveActions.chestGoHomeBehavior);

      GoHomeMessage goHomepelvisMessage = HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.PELVIS, 2);
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

   public void setGrabLocation(Point3D grabLocation, double objectRadius)
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