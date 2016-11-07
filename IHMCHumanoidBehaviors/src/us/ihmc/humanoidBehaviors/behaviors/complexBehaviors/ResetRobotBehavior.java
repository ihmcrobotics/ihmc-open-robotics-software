package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.GoHomeBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.taskExecutor.GoHomeTask;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage.BodyPart;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.taskExecutor.PipeLine;

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

   private final DoubleYoVariable yoTime;

   public ResetRobotBehavior(CommunicationBridge outgoingCommunicationBridge, DoubleYoVariable yoTime)
   {
      this(true,true,true,true,outgoingCommunicationBridge,yoTime);
   }
   public ResetRobotBehavior(boolean leftArm, boolean rightArm, boolean chest, boolean pelvis, CommunicationBridge outgoingCommunicationBridge, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);
      this.leftArm = leftArm;
      this.rightArm = rightArm;
      this.chest = chest;
      this.pelvis = pelvis;
      
      this.yoTime = yoTime;

      chestGoHomeBehavior = new GoHomeBehavior("chest", outgoingCommunicationBridge, yoTime);

      pelvisGoHomeBehavior = new GoHomeBehavior("pelvis", outgoingCommunicationBridge, yoTime);


      armGoHomeLeftBehavior = new GoHomeBehavior("leftArm", outgoingCommunicationBridge, yoTime);
      armGoHomeRightBehavior = new GoHomeBehavior("rightArm", outgoingCommunicationBridge, yoTime);
   }

   @Override
   public void doControl()
   {
      pipeLine.doControl();
   }

   private void setupPipeline()
   {
      
      //RESET BODY POSITIONS *******************************************
      GoHomeMessage goHomeChestMessage = new GoHomeMessage(BodyPart.CHEST, 2);
      chestGoHomeBehavior.setInput(goHomeChestMessage);
      GoHomeTask goHomeChestTask = new GoHomeTask(goHomeChestMessage, chestGoHomeBehavior);

      GoHomeMessage goHomepelvisMessage = new GoHomeMessage(BodyPart.PELVIS, 2);
      pelvisGoHomeBehavior.setInput(goHomepelvisMessage);
      GoHomeTask goHomePelvisTask = new GoHomeTask(goHomepelvisMessage, pelvisGoHomeBehavior);

      GoHomeMessage goHomeLeftArmMessage = new GoHomeMessage(BodyPart.ARM, RobotSide.LEFT, 2);
      armGoHomeLeftBehavior.setInput(goHomeLeftArmMessage);
      GoHomeTask goHomeLeftArmTask = new GoHomeTask(goHomeLeftArmMessage, armGoHomeLeftBehavior);

      GoHomeMessage goHomeRightArmMessage = new GoHomeMessage(BodyPart.ARM, RobotSide.RIGHT, 2);
      armGoHomeRightBehavior.setInput(goHomeRightArmMessage);
      GoHomeTask goHomeRightArmTask = new GoHomeTask(goHomeRightArmMessage, armGoHomeRightBehavior);

      pipeLine.requestNewStage();

      if(rightArm)
      pipeLine.submitSingleTaskStage(goHomeRightArmTask);
      if(leftArm)
      pipeLine.submitSingleTaskStage(goHomeLeftArmTask);
      if(chest)
      pipeLine.submitSingleTaskStage(goHomeChestTask);
      if(pelvis)
      pipeLine.submitSingleTaskStage(goHomePelvisTask);
   }

   
   @Override
   public void initialize()
   {
      super.initialize();
      setupPipeline();
   }

   @Override
   public boolean isDone()
   {
      return pipeLine.isDone();
   }

   
}
