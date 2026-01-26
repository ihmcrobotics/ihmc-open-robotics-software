package us.ihmc.zulu;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.warmup.HumanoidControllerWarmup;
import us.ihmc.avatar.warmup.HumanoidControllerWarmupVisualizer;
import us.ihmc.avatar.warmup.HumanoidControllerWarumupTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;

public class ZuluControllerWarmup extends HumanoidControllerWarmup
{
   public ZuluControllerWarmup()
   {
      this(ZuluVersion.V1_FULL_ROBOT);
   }

   public ZuluControllerWarmup(ZuluVersion version)
   {
      super(new ZuluRobotModel(version, RobotTarget.SCS));
   }

   @Override
   protected void runWarmup()
   {
      HumanoidReferenceFrames referenceFrames = getReferenceFrames();
      FullHumanoidRobotModel fullRobotModel = getFullRobotModel();

      getYoVariable("FootAssumeFootBarelyLoaded").setValueFromDouble(1.0);
      getYoVariable("FootAssumeCopOnEdge").setValueFromDouble(1.0);
      getYoVariable("maxICPErrorBeforeSingleSupportForwardX").setValueFromDouble(Double.POSITIVE_INFINITY);
      getYoVariable("maxICPErrorBeforeSingleSupportBackwardX").setValueFromDouble(Double.POSITIVE_INFINITY);
      getYoVariable("maxICPErrorBeforeSingleSupportInnerY").setValueFromDouble(Double.POSITIVE_INFINITY);
      getYoVariable("maxICPErrorBeforeSingleSupportOuterY").setValueFromDouble(Double.POSITIVE_INFINITY);

      simulate(1.0);
      for (int i = 0; i < 5; i++)
      {
         submitMessage(HumanoidControllerWarumupTools.createStepsInPlace(referenceFrames));
         simulate(1.5);

         for (RobotSide side : RobotSide.values)
         {
            submitMessage(HumanoidControllerWarumupTools.createArmMessage(fullRobotModel, side));
         }
         submitMessage(HumanoidControllerWarumupTools.createChestMessage(referenceFrames));
         simulate(1.0);
      }
   }

   public static void main(String[] args)
   {
      ZuluControllerWarmup atlasControllerWarmup = new ZuluControllerWarmup();
      FloatingRootJointRobot robot = atlasControllerWarmup.getRobotModel().createHumanoidFloatingRootJointRobot(false);
      HumanoidControllerWarmupVisualizer.runAndVisualizeWarmup(atlasControllerWarmup, robot);
   }
}
