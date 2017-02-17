package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ClearLidarBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.EnableLidarBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.GoHomeBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WalkToLocationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.taskExecutor.GoHomeTask;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataStateCommand.LidarState;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage.BodyPart;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.taskExecutor.PipeLine;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

public class BasicPipeLineBehavior extends AbstractBehavior
{
   private final EnableLidarBehavior enableBehaviorOnlyLidarBehavior;
   private final ClearLidarBehavior clearLidarBehavior;
   private final WalkToLocationBehavior walkToLocationBehavior;
   private final GoHomeBehavior armGoHomeLeftBehavior;

   private final PipeLine<AbstractBehavior> pipeLine = new PipeLine<AbstractBehavior>();

   public enum BasicStates
   {
      ENABLE_LIDAR, CLEAR_LIDAR, WALK_TO_LOCATION_AND_HOME_ARM, BEHAVIOR_COMPLETE
   }

   private BasicStates currentState = BasicStates.ENABLE_LIDAR;

   public BasicPipeLineBehavior(String name, DoubleYoVariable yoTime, CommunicationBridge outgoingCommunicationBridge,
         FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames, WholeBodyControllerParameters wholeBodyControllerParameters)
   {
      super(outgoingCommunicationBridge);

      enableBehaviorOnlyLidarBehavior = new EnableLidarBehavior(outgoingCommunicationBridge);
      clearLidarBehavior = new ClearLidarBehavior(outgoingCommunicationBridge);
      walkToLocationBehavior = new WalkToLocationBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames,
            wholeBodyControllerParameters.getWalkingControllerParameters());
      armGoHomeLeftBehavior = new GoHomeBehavior(outgoingCommunicationBridge, yoTime);

      setUpPipeline();
   }

   private void setUpPipeline()
   {
      BehaviorAction enableLidarTask = new BehaviorAction(enableBehaviorOnlyLidarBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            enableBehaviorOnlyLidarBehavior.setLidarState(LidarState.ENABLE_BEHAVIOR_ONLY);
            currentState = BasicStates.ENABLE_LIDAR;
         }
      };

      BehaviorAction clearLidarTask = new BehaviorAction(clearLidarBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            currentState = BasicStates.CLEAR_LIDAR;
         }
      };

      GoHomeMessage goHomeLeftArmMessage = new GoHomeMessage(BodyPart.ARM, RobotSide.LEFT, 2);
      GoHomeTask goHomeLeftArmTask = new GoHomeTask(goHomeLeftArmMessage, armGoHomeLeftBehavior);

      BehaviorAction walkToBallTask = new BehaviorAction(walkToLocationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            currentState = BasicStates.WALK_TO_LOCATION_AND_HOME_ARM;
            FramePose2d poseToWalkTo = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2D(0, 0), 0);
            walkToLocationBehavior.setTarget(poseToWalkTo);
         }

         @Override
         public void doTransitionOutOfAction()
         {

            super.doTransitionOutOfAction();
            currentState = BasicStates.BEHAVIOR_COMPLETE;

         }
      };

      //      FramePose2d poseToWalkTo = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2d(0, 0), 0);
      //
      //      WalkToLocationTask walkToLocationTask = new WalkToLocationTask(poseToWalkTo, walkToLocationBehavior, 0, 2)
      //      {
      //            @Override
      //            public void doTransitionIntoAction()
      //            {
      //               super.doTransitionIntoAction();
      //               currentState = BasicStates.WALK_TO_LOCATION_AND_HOME_ARM;
      //            }
      //      @Override
      //      public void doTransitionOutOfAction()
      //      {
      //       
      //         super.doTransitionOutOfAction();
      //         currentState = BasicStates.BEHAVIOR_COMPLETE;
      //
      //      }
      //      };

      pipeLine.requestNewStage();
      pipeLine.submitSingleTaskStage(enableLidarTask);
      pipeLine.submitSingleTaskStage(clearLidarTask);

      pipeLine.requestNewStage();
      pipeLine.submitTaskForPallelPipesStage(walkToLocationBehavior, walkToBallTask);
      pipeLine.submitTaskForPallelPipesStage(walkToLocationBehavior, goHomeLeftArmTask);

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

   @Override
   public void onBehaviorExited()
   {
   }

  
}
