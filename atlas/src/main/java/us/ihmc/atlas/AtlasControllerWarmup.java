package us.ihmc.atlas;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.warmup.HumanoidControllerWarmup;
import us.ihmc.avatar.warmup.HumanoidControllerWarmupVisualizer;
import us.ihmc.avatar.warmup.HumanoidControllerWarumupTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;

public class AtlasControllerWarmup extends HumanoidControllerWarmup
{
   public AtlasControllerWarmup()
   {
      this(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_FOREARMS);
   }

   public AtlasControllerWarmup(AtlasRobotVersion version)
   {
      super(new AtlasRobotModel(version, RobotTarget.SCS, true));
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
      AtlasControllerWarmup atlasControllerWarmup = new AtlasControllerWarmup();
      FloatingRootJointRobot robot = atlasControllerWarmup.getRobotModel().createHumanoidFloatingRootJointRobot(false);
      HumanoidControllerWarmupVisualizer.runAndVisualizeWarmup(atlasControllerWarmup, robot);
   }
}
