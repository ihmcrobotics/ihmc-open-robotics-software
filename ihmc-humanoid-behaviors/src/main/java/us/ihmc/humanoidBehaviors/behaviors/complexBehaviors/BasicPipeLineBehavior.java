package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import controller_msgs.msg.dds.GoHomeMessage;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ClearLidarBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.EnableLidarBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.GoHomeBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WalkToLocationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.taskExecutor.GoHomeTask;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.HumanoidBodyPart;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.taskExecutor.PipeLine;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoVariables.variable.YoDouble;

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

   public BasicPipeLineBehavior(String robotName, String name, YoDouble yoTime, Ros2Node ros2Node, FullHumanoidRobotModel fullRobotModel,
                                HumanoidReferenceFrames referenceFrames, WholeBodyControllerParameters wholeBodyControllerParameters)
   {
      super(robotName, ros2Node);

      enableBehaviorOnlyLidarBehavior = new EnableLidarBehavior(robotName, ros2Node);
      clearLidarBehavior = new ClearLidarBehavior(robotName, ros2Node);
      walkToLocationBehavior = new WalkToLocationBehavior(robotName, ros2Node, fullRobotModel,
                                                          referenceFrames, wholeBodyControllerParameters.getWalkingControllerParameters());
      armGoHomeLeftBehavior = new GoHomeBehavior(robotName, ros2Node, yoTime);

      setUpPipeline();
   }

   private void setUpPipeline()
   {
      BehaviorAction enableLidarTask = new BehaviorAction(enableBehaviorOnlyLidarBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            // FIXME
            //            enableBehaviorOnlyLidarBehavior.setLidarState(LidarState.ENABLE_BEHAVIOR_ONLY);
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

      GoHomeMessage goHomeLeftArmMessage = HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.ARM, RobotSide.LEFT, 2);
      GoHomeTask goHomeLeftArmTask = new GoHomeTask(goHomeLeftArmMessage, armGoHomeLeftBehavior);

      BehaviorAction walkToZeroTask = new BehaviorAction(walkToLocationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            currentState = BasicStates.WALK_TO_LOCATION_AND_HOME_ARM;
            FramePose2D poseToWalkTo = new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(0, 0), 0);
            walkToLocationBehavior.setTarget(poseToWalkTo);
         }

         @Override
         public void doTransitionOutOfAction()
         {

            super.doTransitionOutOfAction();
            currentState = BasicStates.BEHAVIOR_COMPLETE;

         }
      };

      pipeLine.clearAll();
      pipeLine.requestNewStage();
      pipeLine.submitSingleTaskStage(enableLidarTask);
      pipeLine.submitSingleTaskStage(clearLidarTask);

      pipeLine.requestNewStage();
      pipeLine.submitTaskForPallelPipesStage(walkToLocationBehavior, walkToZeroTask);
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
