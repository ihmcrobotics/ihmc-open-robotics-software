package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WalkToLocationPlannedBehavior;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.taskExecutor.PipeLine;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class TurnInPlaceBehavior extends AbstractBehavior
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final HumanoidReferenceFrames referenceFrames;

   private final YoBoolean hasTargetBeenProvided = new YoBoolean("hasTargetBeenProvided", registry);

   private FramePose3D targetOrientationInWorldFrame;
   private PipeLine<BehaviorAction> pipeLine;
   private final  WalkToLocationPlannedBehavior walkToLocationPlannedBehavior;

   public TurnInPlaceBehavior(String robotName, ROS2Node ros2Node, FullHumanoidRobotModel fullRobotModel,
                              HumanoidReferenceFrames referenceFrames, WalkingControllerParameters walkingControllerParameters, DefaultFootstepPlannerParametersBasics footstepPlannerParameters, YoDouble yoTime )
   {
      super(robotName, ros2Node);
      pipeLine = new PipeLine<>(yoTime);
      walkToLocationPlannedBehavior = new WalkToLocationPlannedBehavior(robotName, ros2Node, fullRobotModel, referenceFrames, walkingControllerParameters,footstepPlannerParameters, yoTime);
      this.referenceFrames = referenceFrames;
      setupPipeline();
   }

   private void setupPipeline()
   {
      pipeLine.clearAll();
      BehaviorAction walk = new BehaviorAction(walkToLocationPlannedBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            walkToLocationPlannedBehavior.setAssumeFlatGround(true);
            walkToLocationPlannedBehavior.setPlanBodyPath(false);
            walkToLocationPlannedBehavior.setPerformAStarSearch(false);
            walkToLocationPlannedBehavior.setTarget(targetOrientationInWorldFrame);
         }
      };
      pipeLine.requestNewStage();
      pipeLine.submitSingleTaskStage(walk);
   }

   @Override
   public void doControl()
   {
      if (!hasTargetBeenProvided.getBooleanValue())
         return;
      pipeLine.doControl();
   }
   
   public void setTarget(double desiredYaw)
   {
      targetOrientationInWorldFrame = new FramePose3D(referenceFrames.getPelvisZUpFrame());
      targetOrientationInWorldFrame.getOrientation().setYawPitchRoll(desiredYaw, 0, 0);
      targetOrientationInWorldFrame.changeFrame(worldFrame);

      hasTargetBeenProvided.set(true);
   }

   @Override
   public void onBehaviorEntered()
   {
      hasTargetBeenProvided.set(false);
      setupPipeline();

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
   public boolean isDone()
   {
     return pipeLine.isDone();
   }

   @Override
   public void onBehaviorExited()
   {
      isPaused.set(false);
      isAborted.set(false);
      hasTargetBeenProvided.set(false);

   }


 
}
