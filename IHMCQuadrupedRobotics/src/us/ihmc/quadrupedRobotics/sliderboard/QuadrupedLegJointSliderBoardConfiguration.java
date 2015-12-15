package us.ihmc.quadrupedRobotics.sliderboard;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.inputdevices.SliderBoardConfigurationManager;

public class QuadrupedLegJointSliderBoardConfiguration
{
   private final String defaultConfiguration = getClass().getSimpleName();

   public QuadrupedLegJointSliderBoardConfiguration(final SliderBoardConfigurationManager sliderBoardConfigurationManager, SimulationConstructionSet scs)
   {
      final EnumYoVariable<RobotQuadrant> sliderBoardMode = new EnumYoVariable<RobotQuadrant>("SliderBoardSelectedRobotQuadrant", scs.getRootRegistry(), RobotQuadrant.class);

      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         //TODO: figure out slider min/max
         String prefix = quadrant.toString().toLowerCase();

         double min = -Math.PI/2;
         double max = Math.PI/2;

         sliderBoardConfigurationManager.setKnob(1, sliderBoardMode, 0.0, 3.0);
         sliderBoardConfigurationManager.setSlider(1, scs.getVariable("QuadrupedLegJointSliderBoardController", prefix + "_hip_roll_q_d"), min, max);
         sliderBoardConfigurationManager.setSlider(2, scs.getVariable("QuadrupedLegJointSliderBoardController", prefix + "_hip_roll_q_d_alpha"), 0.9, 1.0);
         sliderBoardConfigurationManager.setSlider(3, scs.getVariable("QuadrupedLegJointSliderBoardController", prefix + "_hip_pitch_q_d"), min, max);
         sliderBoardConfigurationManager.setSlider(4, scs.getVariable("QuadrupedLegJointSliderBoardController", prefix + "_hip_pitch_q_d_alpha"), 0.9, 1.0);
         sliderBoardConfigurationManager.setSlider(5, scs.getVariable("QuadrupedLegJointSliderBoardController", prefix + "_knee_pitch_q_d"), min, max);
         sliderBoardConfigurationManager.setSlider(6, scs.getVariable("QuadrupedLegJointSliderBoardController", prefix + "_knee_pitch_q_d_alpha"), 0.9, 1.0);
         sliderBoardConfigurationManager.saveConfiguration(quadrant.getCamelCaseNameForStartOfExpression() + defaultConfiguration);
         sliderBoardConfigurationManager.clearControls();
      }

      sliderBoardMode.set(RobotQuadrant.FRONT_LEFT);

      VariableChangedListener listener = new VariableChangedListener()
      {
         @Override public void variableChanged(YoVariable<?> v)
         {
            System.out.println(defaultConfiguration + ": selected " + RobotQuadrant.class.getSimpleName() + " changed to " + sliderBoardMode.getStringValue());
            sliderBoardConfigurationManager.loadConfiguration(sliderBoardMode.getEnumValue().getCamelCaseNameForStartOfExpression() + defaultConfiguration);
         }
      };
      sliderBoardMode.addVariableChangedListener(listener);
      listener.variableChanged(null);
   }

   public String getQuadrantConfiguration(RobotQuadrant robotQuadrant)
   {
      return robotQuadrant.getCamelCaseNameForStartOfExpression() + defaultConfiguration;
   }
}
