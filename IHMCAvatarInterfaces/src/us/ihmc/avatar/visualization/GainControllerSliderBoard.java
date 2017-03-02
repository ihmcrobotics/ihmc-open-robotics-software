package us.ihmc.avatar.visualization;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.CommonNames;
import us.ihmc.simulationconstructionset.util.inputdevices.SliderBoardConfigurationManager;

public class GainControllerSliderBoard
{
   public GainControllerSliderBoard(SimulationConstructionSet scs, YoVariableRegistry registry)
   {
      final SliderBoardConfigurationManager sliderBoardConfigurationManager = new SliderBoardConfigurationManager(scs);



      sliderBoardConfigurationManager.setSlider(1, "carIngressPelvisPositionKp", registry, 0.0, 100.0);
      sliderBoardConfigurationManager.setKnob(1, "carIngressPelvisPositionZeta", registry, 0.0, 1.0);

      sliderBoardConfigurationManager.setSlider(2, "carIngressPelvisOrientationKp", registry, 0.0, 100.0);
      sliderBoardConfigurationManager.setKnob(2, "carIngressPelvisOrientationZeta", registry, 0.0, 1.0);

      sliderBoardConfigurationManager.setSlider(3, "carIngressChestOrientationKp", registry, 0.0, 100.0);
      sliderBoardConfigurationManager.setKnob(3, "carIngressChestOrientationZeta", registry, 0.0, 1.0);

      sliderBoardConfigurationManager.setSlider(4, "carIngressHeadOrientationKp", registry, 0.0, 100.0);
      sliderBoardConfigurationManager.setKnob(4, "carIngressHeadOrientationZeta", registry, 0.0, 1.0);

      sliderBoardConfigurationManager.setSlider(5, "kpAllArmJointsL", registry, 0.0, 100.0);
      sliderBoardConfigurationManager.setKnob(5, "zetaAllArmJointsL", registry, 0.0, 1.0);

      sliderBoardConfigurationManager.setSlider(6, "kpAllArmJointsR", registry, 0.0, 100.0);
      sliderBoardConfigurationManager.setKnob(6, "zetaAllArmJointsR", registry, 0.0, 1.0);

//      sliderBoardConfigurationManager.setSlider(7, "doIHMCControlRatio", registry, 0.0, 1.0);
      sliderBoardConfigurationManager.setSlider(7, CommonNames.doIHMCControlRatio.toString(), registry, 0.0, 1.0);

      // sliderBoardConfigurationManager.saveConfiguration(this.getClass().getSimpleName());
      // sliderBoardConfigurationManager.loadConfiguration(this.getClass().getSimpleName());
   }
}
