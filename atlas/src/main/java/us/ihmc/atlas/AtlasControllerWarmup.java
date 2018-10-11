package us.ihmc.atlas;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.warmup.HumanoidControllerWarmup;
import us.ihmc.avatar.warmup.HumanoidControllerWarmupVisualizer;
import us.ihmc.avatar.warmup.HumanoidControllerWarumupTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;

public class AtlasControllerWarmup extends HumanoidControllerWarmup
{
   private static final AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_FOREARMS, RobotTarget.SCS, true);

   public AtlasControllerWarmup()
   {
      super(robotModel);
   }

   @Override
   protected void runWarmup()
   {
      HumanoidReferenceFrames referenceFrames = getReferenceFrames();
      FullHumanoidRobotModel fullRobotModel = getFullRobotModel();

      for (RobotSide side : RobotSide.values)
      {
         getYoVariable(side.getLowerCaseName() + "FootAssumeFootBarelyLoaded").setValueFromDouble(1.0);
         getYoVariable(side.getLowerCaseName() + "FootAssumeCopOnEdge").setValueFromDouble(1.0);
      }
      getYoVariable("maxICPErrorBeforeSingleSupportX").setValueFromDouble(Double.POSITIVE_INFINITY);
      getYoVariable("maxICPErrorBeforeSingleSupportY").setValueFromDouble(Double.POSITIVE_INFINITY);

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
      FloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      HumanoidControllerWarmupVisualizer.runAndVisualizeWarmup(atlasControllerWarmup, robot);
   }
}
