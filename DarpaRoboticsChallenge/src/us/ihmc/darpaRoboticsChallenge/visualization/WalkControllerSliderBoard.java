package us.ihmc.darpaRoboticsChallenge.visualization;

import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.CommonNames;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.util.inputdevices.SliderBoardConfigurationManager;

public class WalkControllerSliderBoard

{
   public WalkControllerSliderBoard(SimulationConstructionSet scs, YoVariableRegistry registry, GeneralizedSDFRobotModel generalizedSDFRobotModel)
   {
      final EnumYoVariable<SliderBoardMode> sliderBoardMode = new EnumYoVariable<SliderBoardMode>("sliderBoardMode", registry, SliderBoardMode.class);
      final SliderBoardConfigurationManager sliderBoardConfigurationManager = new SliderBoardConfigurationManager(scs);

      sliderBoardConfigurationManager.setSlider(1, "captureKpParallel", registry, 0.0, 2.0);
      sliderBoardConfigurationManager.setKnob(1, "captureKpOrthogonal", registry, 0.0, 2.0);

      sliderBoardConfigurationManager.setSlider(2, "kp_comHeight", registry, 0.0, 40.0);
      sliderBoardConfigurationManager.setKnob(2, "kd_comHeight", registry, 0.0, 13.0);

      sliderBoardConfigurationManager.setSlider(3, "kpPelvisOrientation", registry, 0.0, 100.0);
      sliderBoardConfigurationManager.setKnob(3, "zetaPelvisOrientation", registry, 0.0, 1.0);

//    sliderBoardConfigurationManager.setSlider(4, "walkingHeadOrientationKp", registry, 0.0, 40.0);
//    sliderBoardConfigurationManager.setKnob  (4, "walkingHeadOrientationZeta", registry, 0.0, 1.0);

      sliderBoardConfigurationManager.setSlider(4, "kpUpperBody", registry, 0.0, 200.0);
      sliderBoardConfigurationManager.setKnob(4, "zetaUpperBody", registry, 0.0, 1.0);

      sliderBoardConfigurationManager.setSlider(5, "kpAllArmJointsL", registry, 0.0, 120.0);
      sliderBoardConfigurationManager.setKnob(5, "zetaAllArmJointsL", registry, 0.0, 1.0);

      sliderBoardConfigurationManager.setSlider(6, "kpAllArmJointsR", registry, 0.0, 120.0);
      sliderBoardConfigurationManager.setKnob(6, "zetaAllArmJointsR", registry, 0.0, 1.0);

      sliderBoardConfigurationManager.setSlider(7, CommonNames.doIHMCControlRatio.toString(), registry, 0.0, 1.0);

//    sliderBoardConfigurationManager.setSlider(7, CommonNames.doIHMCControlRatio.toString(), registry, 0.0, 1.0, 3.5, 0.0);
      sliderBoardConfigurationManager.setSlider(8, "offsetHeightAboveGround", registry, 0.0, 0.20);

//    sliderBoardConfigurationManager.setKnob  (8, "sliderBoardMode", registry, 0.0, SliderBoardMode.values().length);
//      sliderBoardConfigurationManager.setKnob(8, "gainScaleFactor", registry, 0.0, 1.0, 3.5, 0.0);

      sliderBoardConfigurationManager.saveConfiguration(SliderBoardMode.WalkingGains.toString());

      sliderBoardConfigurationManager.clearControls();

//      sliderBoardConfigurationManager.setSlider(1, "captureKpParallel", registry, 0.0, 2.0);
//      sliderBoardConfigurationManager.setKnob(1, "captureKpOrthogonal", registry, 0.0, 2.0);

//      sliderBoardConfigurationManager.setSlider(2, "desiredICPX", registry, -0.3, 0.3);
//      sliderBoardConfigurationManager.setKnob(2, "desiredICPY", registry, -0.3, 0.3);
      
      sliderBoardConfigurationManager.setSlider(1, "icpStandOffsetX", registry, -0.3, 0.3);
      sliderBoardConfigurationManager.setKnob(1, "icpStandOffsetY", registry, -0.3, 0.3);
      
      sliderBoardConfigurationManager.setKnob(9, "desiredICPEccentricity", registry, 0, .9);
      sliderBoardConfigurationManager.setKnob(10, "desiredICPAngle", registry, -Math.PI, Math.PI);

//      sliderBoardConfigurationManager.setSlider(3, "hipXJointStiffness", registry, 3000, 20000.0);
//      sliderBoardConfigurationManager.setSlider(4, "hipZJointStiffness", registry, 3000, 20000.0);
//      sliderBoardConfigurationManager.setSlider(5, "hipYJointStiffness", registry, 3000, 20000.0);
//      sliderBoardConfigurationManager.setSlider(6, "otherJointsStiffness", registry, 3000, 20000.0);
      
      sliderBoardConfigurationManager.setKnob(2, "userSetDesiredPelvis", registry, 0.0, 1.0);

      sliderBoardConfigurationManager.setSlider(2, "userDesiredPelvisYaw", registry, -Math.PI, Math.PI);
      sliderBoardConfigurationManager.setSlider(3, "userDesiredPelvisPitch", registry, -0.4, 0.4);
      sliderBoardConfigurationManager.setSlider(4, "userDesiredPelvisRoll", registry, -0.3, 0.3);

//      sliderBoardConfigurationManager.setSlider(5, "userDesiredHeadYaw", registry, -0.8, 0.8);
//
//      sliderBoardConfigurationManager.setKnob(6, "userDesiredHeadPitch", registry, -0.5, 0.5);
//      sliderBoardConfigurationManager.setSlider(6, "userDesiredNeckPitch", registry, -0.8, 0.8);
//      
//      sliderBoardConfigurationManager.setSlider(7, "userDesiredHeadRoll", registry, -0.5, 0.5);
//      sliderBoardConfigurationManager.setKnob(7, "userDesiredNeckRoll", registry, -0.8, 0.8);
      
      sliderBoardConfigurationManager.setSlider(5, "userChestYaw", registry, -0.8, 0.8);
      sliderBoardConfigurationManager.setKnob(5, "userDesiredHeadYaw", registry, -0.8, 0.8);

      sliderBoardConfigurationManager.setSlider(6, "userChestPitch", registry, -0.8, 0.8);
      sliderBoardConfigurationManager.setKnob(6, "userDesiredHeadPitch", registry, -0.5, 0.5);
      
      sliderBoardConfigurationManager.setSlider(7, "userChestRoll", registry, -0.5, 0.5);
      sliderBoardConfigurationManager.setKnob(7, "userDesiredHeadRoll", registry, -0.8, 0.8);

      sliderBoardConfigurationManager.setSlider(8, "offsetHeightAboveGround", registry, 0.0, 0.20);

//    sliderBoardConfigurationManager.setKnob  (8, "sliderBoardMode", registry, 0.0, SliderBoardMode.values().length);
      sliderBoardConfigurationManager.setKnob(8, "gainScaleFactor", registry, 0.0, 1.0, 3.5, 0.0);
      
      sliderBoardConfigurationManager.saveConfiguration(SliderBoardMode.WalkingDesireds.toString());
      sliderBoardConfigurationManager.clearControls();
      
      /* Terrain Exploration Section */
      sliderBoardConfigurationManager.setSlider(1, "footCoPOffsetX", registry, -0.2, 0.2);
      sliderBoardConfigurationManager.setSlider(2, "footCoPOffsetY", registry, -0.1, 0.1);
      sliderBoardConfigurationManager.setButton(1, registry.getVariable("MomentumBasedController","FeetCoPControlIsActive"));
      
      sliderBoardConfigurationManager.saveConfiguration(SliderBoardMode.TerrainExploration.toString());
      sliderBoardConfigurationManager.clearControls();
       
      //default
      sliderBoardMode.set(SliderBoardMode.WalkingGains);

      VariableChangedListener listener = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
               sliderBoardConfigurationManager.loadConfiguration(sliderBoardMode.getEnumValue().toString());
         }
      };

      sliderBoardMode.addVariableChangedListener(listener);
      listener.variableChanged(null);

   }

   private enum SliderBoardMode {WalkingGains, WalkingDesireds, TerrainExploration};

   private static final SliderBoardFactory factory = new SliderBoardFactory()
   {
      @Override
      public void makeSliderBoard(SimulationConstructionSet scs, YoVariableRegistry registry, GeneralizedSDFRobotModel generalizedSDFRobotModel)
      {
         new WalkControllerSliderBoard(scs, registry, generalizedSDFRobotModel);
      }
   };

   public static SliderBoardFactory getFactory()
   {
      return factory;
   }
}
