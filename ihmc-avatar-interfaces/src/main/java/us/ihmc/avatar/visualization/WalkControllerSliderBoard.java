package us.ihmc.avatar.visualization;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.simulationConstructionSetTools.util.inputdevices.SliderBoardConfigurationManager;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.CommonNames;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

public class WalkControllerSliderBoard
{
   private static double MIN_COM_OFFSET_ABOVE_GROUND = 0.0;
   private static double MAX_COM_OFFSET_ABOVE_GROUND = 0.2;

   public WalkControllerSliderBoard(SimulationConstructionSet scs, YoRegistry registry, DRCRobotModel drcRobotModel)
   {
      final YoEnum<SliderBoardMode> sliderBoardMode = new YoEnum<SliderBoardMode>("sliderBoardMode", registry, SliderBoardMode.class);
      final SliderBoardConfigurationManager sliderBoardConfigurationManager = new SliderBoardConfigurationManager(scs);

      // TODO: FIXME: This is a super rough, temporary fix for
      // https://github.com/ihmcrobotics/ihmc-open-robotics-software/issues/26
      boolean DEBUG_WITH_SLIDERBOARD = false;
      if (DEBUG_WITH_SLIDERBOARD)
      {
         sliderBoardConfigurationManager.setSlider(1, "captureKpParallel", registry, 0.0, 2.0);
         sliderBoardConfigurationManager.setKnob(1, "captureKpOrthogonal", registry, 0.0, 2.0);

         sliderBoardConfigurationManager.setSlider(2, "kp_comHeight", registry, 0.0, 40.0);
         sliderBoardConfigurationManager.setKnob(2, "kd_comHeight", registry, 0.0, 13.0);

         sliderBoardConfigurationManager.setSlider(3, "kpPelvisOrientation", registry, 0.0, 100.0);
         sliderBoardConfigurationManager.setKnob(3, "zetaPelvisOrientation", registry, 0.0, 1.0);

         sliderBoardConfigurationManager.setSlider(4, "kpUpperBody", registry, 0.0, 200.0);
         sliderBoardConfigurationManager.setKnob(4, "zetaUpperBody", registry, 0.0, 1.0);

         sliderBoardConfigurationManager.setSlider(5, "kpAllArmJointsL", registry, 0.0, 120.0);
         sliderBoardConfigurationManager.setKnob(5, "zetaAllArmJointsL", registry, 0.0, 1.0);

         sliderBoardConfigurationManager.setSlider(6, "kpAllArmJointsR", registry, 0.0, 120.0);
         sliderBoardConfigurationManager.setKnob(6, "zetaAllArmJointsR", registry, 0.0, 1.0);
      }
      else
      {
         System.out.println("Only claiming sliders 7 and 8 (for tuning change DEBUG_WITH_SLIDERBOARD to true in WalkControllerSliderBoard.java");
      }

      sliderBoardConfigurationManager.setSlider(7, CommonNames.doIHMCControlRatio.toString(), registry, 0.0, 1.0);

      sliderBoardConfigurationManager.setSlider(8, "offsetHeightAboveGround", registry, MIN_COM_OFFSET_ABOVE_GROUND, MAX_COM_OFFSET_ABOVE_GROUND);

      sliderBoardConfigurationManager.saveConfiguration(SliderBoardMode.WalkingGains.toString());

      sliderBoardConfigurationManager.clearControls();

      sliderBoardConfigurationManager.setButton(1, registry.findVariable("PelvisICPBasedTranslationManager", "manualModeICPOffset"));
      sliderBoardConfigurationManager.setSlider(1, "desiredICPOffsetX", registry, -0.3, 0.3);
      sliderBoardConfigurationManager.setKnob(1, "desiredICPOffsetY", registry, -0.3, 0.3);

      sliderBoardConfigurationManager.setButton(2, "userUpdateDesiredPelvisPose", registry);
      sliderBoardConfigurationManager.setButton(3, "userStreamPelvisOrientation", registry);

      sliderBoardConfigurationManager.setSlider(2, "userDesiredPelvisPoseYaw", registry, -0.8, 0.8);
      sliderBoardConfigurationManager.setSlider(3, "userDesiredPelvisPosePitch", registry, -0.4, 0.4);
      sliderBoardConfigurationManager.setSlider(4, "userDesiredPelvisPoseRoll", registry, -0.3, 0.3);

      sliderBoardConfigurationManager.setSlider(5, "userDesiredChestYaw", registry, -0.8, 0.8);
      sliderBoardConfigurationManager.setKnob(5, "userDesiredHeadYaw", registry, -0.8, 0.8);

      sliderBoardConfigurationManager.setSlider(6, "userDesiredChestPitch", registry, -0.8, 0.8);
      sliderBoardConfigurationManager.setKnob(6, "userDesiredHeadPitch", registry, -0.5, 0.5);

      sliderBoardConfigurationManager.setSlider(7, "userDesiredChestRoll", registry, -0.5, 0.5);
      sliderBoardConfigurationManager.setKnob(7, "userDesiredHeadRoll", registry, -0.8, 0.8);

      sliderBoardConfigurationManager.setSlider(8, "offsetHeightAboveGround", registry, 0.0, 0.20);

      //    sliderBoardConfigurationManager.setKnob  (8, "sliderBoardMode", registry, 0.0, SliderBoardMode.values().length);
      sliderBoardConfigurationManager.setKnob(8, "gainScaleFactor", registry, 0.0, 1.0, 3.5, 0.0);

      sliderBoardConfigurationManager.saveConfiguration(SliderBoardMode.WalkingDesireds.toString());
      sliderBoardConfigurationManager.clearControls();

      /* ICPAndCoPFun */
      sliderBoardConfigurationManager.setButton(1, registry.findVariable("PelvisICPBasedTranslationManager", "manualModeICPOffset"));
      sliderBoardConfigurationManager.setSlider(1, "desiredICPOffsetX", registry, -0.6, 0.6);
      sliderBoardConfigurationManager.setKnob(1, "desiredICPOffsetY", registry, -0.6, 0.6);

      sliderBoardConfigurationManager.setSlider(2, "ChestComOffsetX", registry, -0.5, 0.5);
      sliderBoardConfigurationManager.setKnob(2, "ChestComOffsetY", registry, -0.5, 0.5);
      sliderBoardConfigurationManager.setSlider(3, "ChestComOffsetZ", registry, 0.0, 0.5);

      sliderBoardConfigurationManager.setKnob(9, "desiredICPEccentricity", registry, 0, .9);
      sliderBoardConfigurationManager.setKnob(10, "desiredICPAngle", registry, -Math.PI, Math.PI);

      sliderBoardConfigurationManager.saveConfiguration(SliderBoardMode.ICPAndCoPFun.toString());
      sliderBoardConfigurationManager.clearControls();

      /* Terrain Exploration Section */
      sliderBoardConfigurationManager.setSlider(1, "footCoPOffsetX", registry, -0.2, 0.2);
      sliderBoardConfigurationManager.setSlider(2, "footCoPOffsetY", registry, -0.1, 0.1);
      sliderBoardConfigurationManager.setSlider(3, "captureKpParallel", registry, 0.0, 2.0);
      sliderBoardConfigurationManager.setSlider(4, "captureKpOrthogonal", registry, 0.0, 2.0);
      sliderBoardConfigurationManager.setSlider(8, "offsetHeightAboveGround", registry, -0.20, 0.20);

      sliderBoardConfigurationManager.setButton(1, registry.findVariable("MomentumBasedController", "FeetCoPControlIsActive"));

      sliderBoardConfigurationManager.saveConfiguration(SliderBoardMode.TerrainExploration.toString());
      sliderBoardConfigurationManager.clearControls();

      //default
      sliderBoardMode.set(SliderBoardMode.WalkingGains);

      YoVariableChangedListener listener = new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            System.out.println("SliderBoardMode: " + sliderBoardMode.getEnumValue().toString());
            sliderBoardConfigurationManager.loadConfiguration(sliderBoardMode.getEnumValue().toString());
         }
      };

      sliderBoardMode.addListener(listener);
      listener.changed(null);

   }

   private enum SliderBoardMode
   {
      WalkingGains, WalkingDesireds, ICPAndCoPFun, TerrainExploration, HeadJointControl, LeftHandGrasping, RightHandGrasping
   };
}
