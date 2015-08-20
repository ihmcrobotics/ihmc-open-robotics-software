package us.ihmc.darpaRoboticsChallenge.visualization;

import java.util.Arrays;
import java.util.LinkedHashMap;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.CommonNames;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.inputdevices.SliderBoardConfigurationManager;
import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.SdfLoader.partNames.NeckJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoVariable;

public class WalkControllerSliderBoard
{
   private static double MIN_COM_OFFSET_ABOVE_GROUND = 0.0;
   private static double MAX_COM_OFFSET_ABOVE_GROUND = 0.2;
   
   public WalkControllerSliderBoard(SimulationConstructionSet scs, YoVariableRegistry registry, DRCRobotModel drcRobotModel)
   {
      final EnumYoVariable<SliderBoardMode> sliderBoardMode = new EnumYoVariable<SliderBoardMode>("sliderBoardMode", registry, SliderBoardMode.class);
      final SliderBoardConfigurationManager sliderBoardConfigurationManager = new SliderBoardConfigurationManager(scs);

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

      sliderBoardConfigurationManager.setSlider(7, CommonNames.doIHMCControlRatio.toString(), registry, 0.0, 1.0);

      sliderBoardConfigurationManager.setSlider(8, "offsetHeightAboveGround", registry, MIN_COM_OFFSET_ABOVE_GROUND, MAX_COM_OFFSET_ABOVE_GROUND);

      sliderBoardConfigurationManager.saveConfiguration(SliderBoardMode.WalkingGains.toString());

      sliderBoardConfigurationManager.clearControls();

      sliderBoardConfigurationManager.setButton(1, registry.getVariable("PelvisICPBasedTranslationManager", "manualModeICPOffset"));
      sliderBoardConfigurationManager.setSlider(1, "desiredICPOffsetX", registry, -0.3, 0.3);
      sliderBoardConfigurationManager.setKnob(1, "desiredICPOffsetY", registry, -0.3, 0.3);

      sliderBoardConfigurationManager.setKnob(9, "desiredICPEccentricity", registry, 0, .9);
      sliderBoardConfigurationManager.setKnob(10, "desiredICPAngle", registry, -Math.PI, Math.PI);

      //      sliderBoardConfigurationManager.setSlider(3, "hipXJointStiffness", registry, 3000, 20000.0);
      //      sliderBoardConfigurationManager.setSlider(4, "hipZJointStiffness", registry, 3000, 20000.0);
      //      sliderBoardConfigurationManager.setSlider(5, "hipYJointStiffness", registry, 3000, 20000.0);
      //      sliderBoardConfigurationManager.setSlider(6, "otherJointsStiffness", registry, 3000, 20000.0);

      sliderBoardConfigurationManager.setKnob(2, "userSetDesiredPelvis", registry, 0.0, 1.0);

      sliderBoardConfigurationManager.setSlider(2, "userDesiredPelvisYaw", registry, -0.8, 0.8);
      sliderBoardConfigurationManager.setSlider(3, "userDesiredPelvisPitch", registry, -0.4, 0.4);
      sliderBoardConfigurationManager.setSlider(4, "userDesiredPelvisRoll", registry, -0.3, 0.3);

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
      sliderBoardConfigurationManager.setButton(1, registry.getVariable("PelvisICPBasedTranslationManager", "manualModeICPOffset"));
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

      sliderBoardConfigurationManager.setButton(1, registry.getVariable("MomentumBasedController", "FeetCoPControlIsActive"));

      sliderBoardConfigurationManager.saveConfiguration(SliderBoardMode.TerrainExploration.toString());
      sliderBoardConfigurationManager.clearControls();

      if (drcRobotModel != null && drcRobotModel.getWalkingControllerParameters().controlHeadAndHandsWithSliders())
      {
         setupHeadAndHandSliders(sliderBoardConfigurationManager, sliderBoardMode, drcRobotModel, registry);
         setupIndividualHandControl(sliderBoardConfigurationManager, sliderBoardMode, drcRobotModel, registry);
      }

      //default
      sliderBoardMode.set(SliderBoardMode.WalkingGains);

      VariableChangedListener listener = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            System.out.println("SliderBoardMode: " + sliderBoardMode.getEnumValue().toString());
            sliderBoardConfigurationManager.loadConfiguration(sliderBoardMode.getEnumValue().toString());
         }
      };

      sliderBoardMode.addVariableChangedListener(listener);
      listener.variableChanged(null);

   }

   private void setupIndividualHandControl(final SliderBoardConfigurationManager sliderBoardConfigurationManager,
         final EnumYoVariable<SliderBoardMode> sliderBoardMode, final DRCRobotModel drcRobotModel, final YoVariableRegistry registry)
   {
      sliderBoardConfigurationManager.setKnob(1, sliderBoardMode, 0, sliderBoardMode.getEnumValues().length - 1);
      SideDependentList<LinkedHashMap<String, ImmutablePair<Double, Double>>> actuatableFingerJointsWithLimits = drcRobotModel.getSliderBoardControlledFingerJointsWithLimits();
      //This currently assumes you don't have more than 8 actuatable finger joints per hand because the sliderboard only has 8 sliders. 
      for (RobotSide side : RobotSide.values())
      {
         int i = 0;
         for (String actuatableFingerJointName : actuatableFingerJointsWithLimits.get(side).keySet())
         {
            sliderBoardConfigurationManager.setSlider(++i, actuatableFingerJointName + CommonNames.q_d.toString(), registry,
            actuatableFingerJointsWithLimits.get(side).get(actuatableFingerJointName).getLeft(), actuatableFingerJointsWithLimits.get(side).get(actuatableFingerJointName)
                  .getRight());
         }
         if (side == RobotSide.LEFT)
         {
            sliderBoardConfigurationManager.saveConfiguration(SliderBoardMode.LeftHandGrasping.toString());
         }
         else
         {
            sliderBoardConfigurationManager.saveConfiguration(SliderBoardMode.RightHandGrasping.toString());
         }
         sliderBoardConfigurationManager.clearControls();
      }
   }

   private void setupHeadAndHandSliders(final SliderBoardConfigurationManager sliderBoardConfigurationManager,
         final EnumYoVariable<SliderBoardMode> sliderBoardMode, final DRCRobotModel drcRobotModel, final YoVariableRegistry registry)
   {

      //Make sure the joints you want to control are not being controlled by any other control module.
      final DoubleYoVariable alpha = new DoubleYoVariable("HeadSliderControlAlpha", registry);
      alpha.set(0.6);
      final DoubleYoVariable headYawPercentage = new DoubleYoVariable("SliderHeadYawPercentage", registry);
      final DoubleYoVariable lowerHeadPitchPercentage = new DoubleYoVariable("SliderLowerHeadPitchPercentage", registry);
      final DoubleYoVariable upperHeadPitchPercentage = new DoubleYoVariable("SliderUpperHeadPitchPercentage", registry);
      final AlphaFilteredYoVariable alphaFilteredHeadYawPercentage = new AlphaFilteredYoVariable("AlphaFilteredHeadYawPercentage",registry,alpha,headYawPercentage);
      final AlphaFilteredYoVariable alphaFilteredUpperHeadPitchPercentage = new AlphaFilteredYoVariable("AlphaFilteredUpperHeadPitchPercentage",registry,alpha,upperHeadPitchPercentage);
      final AlphaFilteredYoVariable alphaFilteredLowerHeadPitchYawPercentage = new AlphaFilteredYoVariable("AlphaFilteredLowerHeadPitchPercentage",registry,alpha,lowerHeadPitchPercentage);
      final LinkedHashMap<NeckJointName, ImmutablePair<Double, Double>> sliderBoardControlledNeckJointsWithLimits = drcRobotModel
            .getSliderBoardControlledNeckJointsWithLimits();
      int sliderNumber = 0;

      if (Arrays.asList(drcRobotModel.getJointMap().getNeckJointNames()).contains(NeckJointName.NECK_YAW))
      {
         double standPrepAngle = drcRobotModel.getStandPrepAngle(drcRobotModel.getJointMap().getNeckJointName(NeckJointName.NECK_YAW));
         double neckYawJointRange = Math.abs(sliderBoardControlledNeckJointsWithLimits.get(NeckJointName.NECK_YAW).getRight()
               - sliderBoardControlledNeckJointsWithLimits.get(NeckJointName.NECK_YAW).getLeft());
         double standPrepPercentage = Math.abs(standPrepAngle - sliderBoardControlledNeckJointsWithLimits.get(NeckJointName.NECK_YAW).getLeft())
               / neckYawJointRange;
         sliderBoardConfigurationManager.setKnob(1, headYawPercentage, 0.0, 1.0);
         headYawPercentage.set(standPrepPercentage);
         alphaFilteredHeadYawPercentage.set(headYawPercentage.getDoubleValue());
      }
      if (Arrays.asList(drcRobotModel.getJointMap().getNeckJointNames()).contains(NeckJointName.LOWER_NECK_PITCH))
      {
         double standPrepAngle = drcRobotModel.getStandPrepAngle(drcRobotModel.getJointMap().getNeckJointName(NeckJointName.LOWER_NECK_PITCH));
         double lowerNeckPitchJointRange = Math.abs(sliderBoardControlledNeckJointsWithLimits.get(NeckJointName.LOWER_NECK_PITCH).getRight()
               - sliderBoardControlledNeckJointsWithLimits.get(NeckJointName.LOWER_NECK_PITCH).getLeft());
         double standPrepPercentage = Math.abs(standPrepAngle - sliderBoardControlledNeckJointsWithLimits.get(NeckJointName.LOWER_NECK_PITCH).getLeft())
               / lowerNeckPitchJointRange;
         sliderBoardConfigurationManager.setSlider(++sliderNumber, lowerHeadPitchPercentage, 0.0, 1.0);
         lowerHeadPitchPercentage.set(standPrepPercentage);
         alphaFilteredLowerHeadPitchYawPercentage.set(lowerHeadPitchPercentage.getDoubleValue());
      }
      if (Arrays.asList(drcRobotModel.getJointMap().getNeckJointNames()).contains(NeckJointName.UPPER_NECK_PITCH))
      {
         double standPrepAngle = drcRobotModel.getStandPrepAngle(drcRobotModel.getJointMap().getNeckJointName(NeckJointName.UPPER_NECK_PITCH));
         double upperNeckPitchJointRange = Math.abs(sliderBoardControlledNeckJointsWithLimits.get(NeckJointName.UPPER_NECK_PITCH).getRight()
               - sliderBoardControlledNeckJointsWithLimits.get(NeckJointName.UPPER_NECK_PITCH).getLeft());
         double standPrepPercentage = Math.abs(standPrepAngle - sliderBoardControlledNeckJointsWithLimits.get(NeckJointName.UPPER_NECK_PITCH).getLeft())
               / upperNeckPitchJointRange;
         sliderBoardConfigurationManager.setSlider(++sliderNumber, upperHeadPitchPercentage, 0.0, 1.0);
         upperHeadPitchPercentage.set(standPrepPercentage);
         alphaFilteredUpperHeadPitchPercentage.set(upperHeadPitchPercentage.getDoubleValue());
      }

      sliderBoardConfigurationManager.saveConfiguration(SliderBoardMode.HeadJointControl.toString());
      sliderBoardConfigurationManager.clearControls();

      if (Arrays.asList(drcRobotModel.getJointMap().getNeckJointNames()).contains(NeckJointName.NECK_YAW))
      {
         headYawPercentage.addVariableChangedListener(new VariableChangedListener()
         {
            @Override
            public void variableChanged(YoVariable<?> v)
            {
               alphaFilteredHeadYawPercentage.update(headYawPercentage.getDoubleValue());
               NeckJointName headYaw = NeckJointName.NECK_YAW;
               double neckYawJointRange = sliderBoardControlledNeckJointsWithLimits.get(headYaw).getRight()
                     - sliderBoardControlledNeckJointsWithLimits.get(headYaw).getLeft();
               DoubleYoVariable desiredAngle = (DoubleYoVariable) registry.getVariable(drcRobotModel.getJointMap().getNeckJointName(headYaw) + "_unconstrained"
                     + CommonNames.q_d);
               desiredAngle.set(alphaFilteredHeadYawPercentage.getDoubleValue() * neckYawJointRange + sliderBoardControlledNeckJointsWithLimits.get(headYaw).getLeft());
            }
         });
      }

      if (Arrays.asList(drcRobotModel.getJointMap().getNeckJointNames()).contains(NeckJointName.UPPER_NECK_PITCH))
      {
         upperHeadPitchPercentage.addVariableChangedListener(new VariableChangedListener()
         {
            @Override
            public void variableChanged(YoVariable<?> v)
            {
               alphaFilteredUpperHeadPitchPercentage.update(upperHeadPitchPercentage.getDoubleValue());
               NeckJointName upperHeadPitch = NeckJointName.UPPER_NECK_PITCH;
               double jointRange = sliderBoardControlledNeckJointsWithLimits.get(upperHeadPitch).getRight()
                     - sliderBoardControlledNeckJointsWithLimits.get(upperHeadPitch).getLeft();
               DoubleYoVariable desiredAngle = (DoubleYoVariable) registry.getVariable(drcRobotModel.getJointMap().getNeckJointName(upperHeadPitch)
                     + "_unconstrained" + CommonNames.q_d);
               desiredAngle.set(alphaFilteredUpperHeadPitchPercentage.getDoubleValue() * jointRange + sliderBoardControlledNeckJointsWithLimits.get(upperHeadPitch).getLeft());
            }
         });
      }

      if (Arrays.asList(drcRobotModel.getJointMap().getNeckJointNames()).contains(NeckJointName.LOWER_NECK_PITCH))
      {
         lowerHeadPitchPercentage.addVariableChangedListener(new VariableChangedListener()
         {
            @Override
            public void variableChanged(YoVariable<?> v)
            {
               alphaFilteredLowerHeadPitchYawPercentage.update(lowerHeadPitchPercentage.getDoubleValue());
               NeckJointName lowerHeadPitch = NeckJointName.LOWER_NECK_PITCH;
               double jointRange = sliderBoardControlledNeckJointsWithLimits.get(lowerHeadPitch).getRight()
                     - sliderBoardControlledNeckJointsWithLimits.get(lowerHeadPitch).getLeft();
               DoubleYoVariable desiredAngle = (DoubleYoVariable) registry.getVariable(drcRobotModel.getJointMap().getNeckJointName(lowerHeadPitch)
                     + "_unconstrained" + CommonNames.q_d);
               desiredAngle.set(alphaFilteredLowerHeadPitchYawPercentage.getDoubleValue() * jointRange + sliderBoardControlledNeckJointsWithLimits.get(lowerHeadPitch).getLeft());
            }
         });
      }
   }

   private enum SliderBoardMode
   {
      WalkingGains, WalkingDesireds, ICPAndCoPFun, TerrainExploration, HeadJointControl, LeftHandGrasping, RightHandGrasping
   };
}
