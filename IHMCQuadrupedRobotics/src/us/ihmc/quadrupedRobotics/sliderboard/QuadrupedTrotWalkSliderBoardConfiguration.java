package us.ihmc.quadrupedRobotics.sliderboard;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.inputdevices.SliderBoardConfigurationManager;

public class QuadrupedTrotWalkSliderBoardConfiguration
{
   private final String trotWalk = SliderBoardModes.TROT_WALK.toString();
   
   public QuadrupedTrotWalkSliderBoardConfiguration(final SliderBoardConfigurationManager sliderBoardConfigurationManager, SimulationConstructionSet scs)
   {
      final EnumYoVariable<SliderBoardModes> selectedMode = (EnumYoVariable<SliderBoardModes>) scs.getRootRegistry().getVariable("sliderboardMode");
      
      selectedMode.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            SliderBoardModes sliderBoardMode = SliderBoardModes.values()[selectedMode.getOrdinal()];
            System.out.println("loading configuration " + sliderBoardMode);
            sliderBoardConfigurationManager.loadConfiguration(sliderBoardMode.toString());
         }
      });
      
      sliderBoardConfigurationManager.setSlider(1, "q_d_z", scs, 0.4, 1.0);
      sliderBoardConfigurationManager.setSlider(2, "q_d_x", scs, -0.1, 0.1);
      sliderBoardConfigurationManager.setSlider(3, "q_d_y", scs, -0.1, 0.1);
      sliderBoardConfigurationManager.setSlider(4, "q_d_roll", scs, -0.2, 0.2);
      sliderBoardConfigurationManager.setSlider(5, "q_d_pitch", scs, -0.1, 0.1);
      sliderBoardConfigurationManager.setSlider(6, "q_d_yaw", scs, -0.03, 0.03);
      sliderBoardConfigurationManager.saveConfiguration(trotWalk);
   }
   
   public String getDefaultConfiguration()
   {
      return trotWalk;
   }
}
