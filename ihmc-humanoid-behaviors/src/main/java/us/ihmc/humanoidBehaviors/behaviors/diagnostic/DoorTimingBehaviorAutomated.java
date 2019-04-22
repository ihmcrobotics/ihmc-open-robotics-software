package us.ihmc.humanoidBehaviors.behaviors.diagnostic;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.WalkThroughDoorBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.taskExecutor.PipeLine;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class DoorTimingBehaviorAutomated extends AbstractBehavior
{
   private final PipeLine<AbstractBehavior> pipeLine = new PipeLine<>();

   private final WalkThroughDoorBehavior walkThroughDoorBehavior;
   private final DoorTimingBehavior doorTimingBehavior;


   public DoorTimingBehaviorAutomated(String robotName, Ros2Node ros2Node, YoDouble yoTime, YoBoolean yoDoubleSupport, FullHumanoidRobotModel fullRobotModel,
                                            HumanoidReferenceFrames referenceFrames, WholeBodyControllerParameters wholeBodyControllerParameters,
                                            AtlasPrimitiveActions atlasPrimitiveActions, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(robotName, ros2Node);
      walkThroughDoorBehavior = new WalkThroughDoorBehavior(robotName, "automated", ros2Node, yoTime, yoDoubleSupport, fullRobotModel, referenceFrames, wholeBodyControllerParameters, atlasPrimitiveActions, yoGraphicsListRegistry);
      doorTimingBehavior = new DoorTimingBehavior(robotName, yoTime, ros2Node, false);
   }

   @Override
   public void onBehaviorEntered()
   {
      setupPipeline();
   }

   private void setupPipeline()
   {
      pipeLine.clearAll();

      BehaviorAction walkThroughDoor = new BehaviorAction(walkThroughDoorBehavior,doorTimingBehavior);
      pipeLine.requestNewStage();
      pipeLine.submitSingleTaskStage(walkThroughDoor);
      
     // pipeLine.submitTaskForPallelPipesStage(doorTimingBehavior, timingBehavior);
   }


   @Override
   public void onBehaviorExited()
   {
      
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