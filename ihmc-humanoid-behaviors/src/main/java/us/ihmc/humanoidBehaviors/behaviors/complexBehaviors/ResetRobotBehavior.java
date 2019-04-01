package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import controller_msgs.msg.dds.GoHomeMessage;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.GoHomeBehavior;
import us.ihmc.humanoidBehaviors.taskExecutor.GoHomeTask;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.HumanoidBodyPart;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.taskExecutor.PipeLine;
import us.ihmc.yoVariables.variable.YoDouble;

public class ResetRobotBehavior extends AbstractBehavior
{
   private final GoHomeBehavior chestGoHomeBehavior;
   private final GoHomeBehavior pelvisGoHomeBehavior;
   private final GoHomeBehavior armGoHomeLeftBehavior;
   private final GoHomeBehavior armGoHomeRightBehavior;

   boolean leftArm = true;
   boolean rightArm = true;
   boolean chest = true;
   boolean pelvis = true;

   private final PipeLine<AbstractBehavior> pipeLine = new PipeLine<>();

   public ResetRobotBehavior(String robotName, Ros2Node ros2Node, YoDouble yoTime)
   {
      this(robotName, true, true, true, true, ros2Node, yoTime);
   }

   public ResetRobotBehavior(String robotName, boolean leftArm, boolean rightArm, boolean chest, boolean pelvis, Ros2Node ros2Node, YoDouble yoTime)
   {
      super(robotName, ros2Node);
      this.leftArm = leftArm;
      this.rightArm = rightArm;
      this.chest = chest;
      this.pelvis = pelvis;

      chestGoHomeBehavior = new GoHomeBehavior(robotName, "chest", ros2Node, yoTime);

      pelvisGoHomeBehavior = new GoHomeBehavior(robotName, "pelvis", ros2Node, yoTime);

      armGoHomeLeftBehavior = new GoHomeBehavior(robotName, "leftArm", ros2Node, yoTime);
      armGoHomeRightBehavior = new GoHomeBehavior(robotName, "rightArm", ros2Node, yoTime);
   }

   @Override
   public void doControl()
   {
      if (!isPaused())
      pipeLine.doControl();
   }

   private void setupPipeline()
   {

      publishTextToSpeech("Resetting Robot Pose");
      pipeLine.clearAll();
      //RESET BODY POSITIONS *******************************************
      GoHomeMessage goHomeChestMessage = HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.CHEST, 2);
      GoHomeTask goHomeChestTask = new GoHomeTask(goHomeChestMessage, chestGoHomeBehavior);

      GoHomeMessage goHomepelvisMessage = HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.PELVIS, 2);
      GoHomeTask goHomePelvisTask = new GoHomeTask(goHomepelvisMessage, pelvisGoHomeBehavior);

      GoHomeMessage goHomeLeftArmMessage = HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.ARM, RobotSide.LEFT, 2);
      GoHomeTask goHomeLeftArmTask = new GoHomeTask(goHomeLeftArmMessage, armGoHomeLeftBehavior);

      GoHomeMessage goHomeRightArmMessage = HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.ARM, RobotSide.RIGHT, 2);
      GoHomeTask goHomeRightArmTask = new GoHomeTask(goHomeRightArmMessage, armGoHomeRightBehavior);

      pipeLine.requestNewStage();

      if (rightArm)
         pipeLine.submitSingleTaskStage(goHomeRightArmTask);
      if (leftArm)
         pipeLine.submitSingleTaskStage(goHomeLeftArmTask);
      if (chest)
         pipeLine.submitSingleTaskStage(goHomeChestTask);
      if (pelvis)
         pipeLine.submitSingleTaskStage(goHomePelvisTask);
   }

   @Override
   public void onBehaviorEntered()
   {
      setupPipeline();
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

   @Override
   public void onBehaviorExited()
   {
   }

}
