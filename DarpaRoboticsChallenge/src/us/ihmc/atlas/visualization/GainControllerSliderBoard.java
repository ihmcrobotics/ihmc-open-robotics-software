package us.ihmc.atlas.visualization;

import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.inputdevices.SliderBoardConfigurationManager;

public class GainControllerSliderBoard extends SliderBoard {

   public GainControllerSliderBoard(SimulationConstructionSet scs, YoVariableRegistry registry, GeneralizedSDFRobotModel generalizedSDFRobotModel)
   {
  
      
      final SliderBoardConfigurationManager sliderBoardConfigurationManager = new SliderBoardConfigurationManager(scs);
      


      sliderBoardConfigurationManager.setSlider(1, "carIngressPelvisPositionKp", registry, 0.0, 100.0);
      sliderBoardConfigurationManager.setKnob  (1, "carIngressPelvisPositionZeta", registry, 0.0, 1.0);

      sliderBoardConfigurationManager.setSlider(2, "carIngressPelvisOrientationKp", registry, 0.0, 100.0);
      sliderBoardConfigurationManager.setKnob  (2, "carIngressPelvisOrientationZeta", registry, 0.0, 1.0);

      sliderBoardConfigurationManager.setSlider(3, "carIngressChestOrientationKp", registry, 0.0, 100.0);
      sliderBoardConfigurationManager.setKnob  (3, "carIngressChestOrientationZeta", registry, 0.0, 1.0);

      sliderBoardConfigurationManager.setSlider(4, "carIngressHeadOrientationKp", registry, 0.0, 100.0);
      sliderBoardConfigurationManager.setKnob  (4, "carIngressHeadOrientationZeta", registry, 0.0, 1.0);

      sliderBoardConfigurationManager.setSlider(5, "kpAllArmJointsLeft", registry, 0.0, 100.0);
      sliderBoardConfigurationManager.setKnob  (5, "zetaAllArmJointsLeft", registry, 0.0, 1.0);

      sliderBoardConfigurationManager.setSlider(6, "kpAllArmJointsRight", registry, 0.0, 100.0);
      sliderBoardConfigurationManager.setKnob  (6, "zetaAllArmJointsRight", registry, 0.0, 1.0);

      
      //sliderBoardConfigurationManager.saveConfiguration(this.getClass().getSimpleName());
      //sliderBoardConfigurationManager.loadConfiguration(this.getClass().getSimpleName());
   }
   
   public static SliderBoardFactory factory = new SliderBoardFactory() {
	
		@Override
		public SliderBoard makeSliderBoard(SimulationConstructionSet scs, YoVariableRegistry registry, GeneralizedSDFRobotModel generalizedSDFRobotModel) {
			   return new GainControllerSliderBoard( scs,  registry,  generalizedSDFRobotModel);
		}
   };
   
   
}
