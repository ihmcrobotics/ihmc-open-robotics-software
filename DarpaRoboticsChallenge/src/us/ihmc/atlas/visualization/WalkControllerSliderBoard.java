package us.ihmc.atlas.visualization;

import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;

import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.VariableChangedListener;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.inputdevices.SliderBoardConfigurationManager;

public class WalkControllerSliderBoard

{

   public WalkControllerSliderBoard(SimulationConstructionSet scs, YoVariableRegistry registry, GeneralizedSDFRobotModel generalizedSDFRobotModel)
   {
      final EnumYoVariable<SliderBoardMode> sliderBoardMode = new EnumYoVariable<SliderBoardMode>("sliderBoardMode", registry, SliderBoardMode.class);
      final SliderBoardConfigurationManager sliderBoardConfigurationManager = new SliderBoardConfigurationManager(scs);
      
      sliderBoardConfigurationManager.setSlider(1, "captureKpParallel", registry, 0.0, 2.0);
      sliderBoardConfigurationManager.setKnob  (1, "captureKpOrthogonal", registry, 0.0, 2.0);

      sliderBoardConfigurationManager.setSlider(2, "kp_comHeight", registry, 0.0, 40.0);
      sliderBoardConfigurationManager.setKnob  (2, "kd_comHeight", registry, 0.0, 13.0);

      sliderBoardConfigurationManager.setSlider(3, "kpPelvisOrientation", registry, 0.0, 100.0);
      sliderBoardConfigurationManager.setKnob  (3, "zetaPelvisOrientation", registry, 0.0, 1.0);

//      sliderBoardConfigurationManager.setSlider(4, "walkingHeadOrientationKp", registry, 0.0, 40.0);
//      sliderBoardConfigurationManager.setKnob  (4, "walkingHeadOrientationZeta", registry, 0.0, 1.0);
      
      sliderBoardConfigurationManager.setSlider(4, "kpUpperBody", registry, 0.0, 200.0);
      sliderBoardConfigurationManager.setKnob  (4, "zetaUpperBody", registry, 0.0, 1.0);

      sliderBoardConfigurationManager.setSlider(5, "kpAllArmJointsL", registry, 0.0, 120.0);
      sliderBoardConfigurationManager.setKnob  (5, "zetaAllArmJointsL", registry, 0.0, 1.0);

      sliderBoardConfigurationManager.setSlider(6, "kpAllArmJointsR", registry, 0.0, 120.0);
      sliderBoardConfigurationManager.setKnob  (6, "zetaAllArmJointsR", registry, 0.0, 1.0);

      sliderBoardConfigurationManager.setSlider(7, "ll_transitionRatio", registry,  0.0, 1.0);
      sliderBoardConfigurationManager.setSlider(8, "offsetHeightAboveGround", registry, 0.0, 0.12);
      
      sliderBoardConfigurationManager.setKnob  (8, "sliderBoardMode", registry, 0.0, SliderBoardMode.values().length);

      sliderBoardConfigurationManager.saveConfiguration(SliderBoardMode.WalkingGains.toString());
      sliderBoardConfigurationManager.clearControls();
      
      sliderBoardConfigurationManager.setSlider(1, "offsetHeightAboveGround", registry, 0.0, 0.12);

      sliderBoardConfigurationManager.setSlider(2, "desiredICPX", registry, -0.05, 0.15);
      sliderBoardConfigurationManager.setKnob  (2, "desiredICPY", registry, -0.2, 0.2);
      
      sliderBoardConfigurationManager.setSlider(3, "userDesiredPelvisYaw", registry, -Math.PI, Math.PI);
      sliderBoardConfigurationManager.setKnob  (3, "userSetDesiredPelvis", registry, 0.0, 1.0);
      sliderBoardConfigurationManager.setSlider(4, "userDesiredPelvisPitch", registry, -0.4, 0.4);
      sliderBoardConfigurationManager.setSlider(5, "userDesiredPelvisRoll", registry, -0.3, 0.3);
      
      sliderBoardConfigurationManager.setSlider(6, "userDesiredHeadPitch", registry, -0.5, 0.5);
      sliderBoardConfigurationManager.setSlider(7, "userDesiredHeadYaw", registry, -0.8, 0.8);

      sliderBoardConfigurationManager.setKnob  (8, "sliderBoardMode", registry, 0.0, SliderBoardMode.values().length);

      sliderBoardConfigurationManager.saveConfiguration(SliderBoardMode.WalkingDesireds.toString());
      sliderBoardConfigurationManager.clearControls();

      
      sliderBoardMode.set(SliderBoardMode.WalkingGains);
      
      VariableChangedListener listener = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable v)
         {
            sliderBoardConfigurationManager.loadConfiguration(sliderBoardMode.getEnumValue().toString());
         }
      };
      
      sliderBoardMode.addVariableChangedListener(listener);
      listener.variableChanged(null);
      
   }
   
   private enum SliderBoardMode
   {
      WalkingGains, WalkingDesireds;
   }
   
   private static final SliderBoardFactory factory = new SliderBoardFactory() {
   
      @Override
      public void makeSliderBoard(SimulationConstructionSet scs, YoVariableRegistry registry, GeneralizedSDFRobotModel generalizedSDFRobotModel) 
      {
         new WalkControllerSliderBoard( scs,  registry,  generalizedSDFRobotModel);
      }
   };
   
   public static SliderBoardFactory getFactory()
   {
      return factory;
   }
}
