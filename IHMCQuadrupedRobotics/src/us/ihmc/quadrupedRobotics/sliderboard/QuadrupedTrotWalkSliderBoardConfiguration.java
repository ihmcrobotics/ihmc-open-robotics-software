package us.ihmc.quadrupedRobotics.sliderboard;

import us.ihmc.quadrupedRobotics.controller.state.QuadrupedControllerState;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.inputdevices.EnumDependentSliderBoardMapping;
import us.ihmc.simulationconstructionset.util.inputdevices.SliderBoardConfigurationManager;

public class QuadrupedTrotWalkSliderBoardConfiguration implements EnumDependentSliderBoardMapping<QuadrupedControllerState>
{
   private final EnumYoVariable<QuadrupedSliderBoardMode> sliderboardMode;
   
   public QuadrupedTrotWalkSliderBoardConfiguration(final SliderBoardConfigurationManager sliderBoardConfigurationManager, SimulationConstructionSet scs)
   {
      sliderboardMode = QuadrupedSliderBoardMode.getYoVariable(scs.getRootRegistry());
      sliderboardMode.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            QuadrupedSliderBoardMode sliderBoardMode = QuadrupedSliderBoardMode.values()[sliderboardMode.getOrdinal()];
            System.out.println("loading configuration " + sliderBoardMode);
            sliderBoardConfigurationManager.loadConfiguration(sliderBoardMode.toString());
         }
      });
      
      String namespace = "root.babyBeastSimple.QuadrupedSimulationController.QuadrupedControllerManager.TrotWalkController.";
      sliderBoardConfigurationManager.setSlider(1, namespace + "desiredStanceZ", scs, 0.4, 0.7);
      sliderBoardConfigurationManager.setSlider(2, namespace + "desiredStanceX", scs, -0.2, 0.2);
      sliderBoardConfigurationManager.setSlider(3, namespace + "desiredStanceY", scs, -0.2, 0.2);
      sliderBoardConfigurationManager.setSlider(4, namespace + "desiredStanceRoll", scs, -0.2, 0.2);
      sliderBoardConfigurationManager.setSlider(5, namespace + "desiredStancePitch", scs, -0.1, 0.1);
      sliderBoardConfigurationManager.setSlider(6, namespace + "desiredStanceYaw", scs, -0.1, 0.1);
      sliderBoardConfigurationManager.setSlider(7, namespace + "desiredICPFromCentroidX", scs, -0.2, 0.2);
      sliderBoardConfigurationManager.setSlider(8, namespace + "desiredICPFromCentroidY", scs, -0.2, 0.2);
      sliderBoardConfigurationManager.setButton(1, namespace + "enableTrot", scs);
      sliderBoardConfigurationManager.saveConfiguration(QuadrupedSliderBoardMode.TROT_WALK.toString());
   }

   @Override
   public QuadrupedControllerState getEnum()
   {
      return QuadrupedControllerState.TROT_WALK;
   }

   @Override
   public void activateConfiguration(SliderBoardConfigurationManager sliderBoardConfigurationManager)
   {
      sliderboardMode.set(QuadrupedSliderBoardMode.TROT_WALK);
      sliderBoardConfigurationManager.loadConfiguration(QuadrupedSliderBoardMode.TROT_WALK.toString());
   }
}
