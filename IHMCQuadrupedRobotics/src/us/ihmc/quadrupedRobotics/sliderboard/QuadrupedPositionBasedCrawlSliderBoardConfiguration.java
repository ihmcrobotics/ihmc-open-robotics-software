package us.ihmc.quadrupedRobotics.sliderboard;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.inputdevices.SliderBoardConfigurationManager;

public class QuadrupedPositionBasedCrawlSliderBoardConfiguration
{
   private final String positionCrawlCoMShift = SliderBoardModes.POSITIONCRAWL_COM_SHIFT.toString();
   private final String positionCrawlFootstepChooser = SliderBoardModes.POSITIONCRAWL_FOOTSTEP_CHOOSER.toString();
   private final String positionCrawlOrientationTuning = SliderBoardModes.POSITIONCRAWL_ORIENTATION_TUNING.toString();
   
   public QuadrupedPositionBasedCrawlSliderBoardConfiguration(final SliderBoardConfigurationManager sliderBoardConfigurationManager, SimulationConstructionSet scs)
   {
      final EnumYoVariable<SliderBoardModes> selectedMode = (EnumYoVariable<SliderBoardModes>) scs.getRootRegistry().getVariable("sliderboardMode");
      
      selectedMode.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            System.out.println("loading configuration " + selectedMode.getEnumValue());
            sliderBoardConfigurationManager.loadConfiguration(selectedMode.getEnumValue().toString());
         }
      });
      
      //CoM Shift tuning
      sliderBoardConfigurationManager.setSlider(1, "userProvidedDesiredVelocityX", scs, -0.4, 0.4);
      sliderBoardConfigurationManager.setSlider(2, "userProvidedDesiredVelocityY", scs, -0.5, 0.5);
      sliderBoardConfigurationManager.setSlider(3, "userProvidedDesiredYawRate", scs, -0.3, 0.3);
      sliderBoardConfigurationManager.setSlider(4, "desiredCoMHeight", scs, 0.0, 1.0);
      sliderBoardConfigurationManager.setSlider(5, "filteredDesiredCoMHeightAlphaBreakFrequency", scs, 0.0, 1.0);
      sliderBoardConfigurationManager.setSlider(6, "subCircleRadius", scs, 0.0, 0.3);
      sliderBoardConfigurationManager.setSliderEnum(8, selectedMode);
      sliderBoardConfigurationManager.saveConfiguration(positionCrawlCoMShift);
      
      
      //footstep chooser tuning
      sliderBoardConfigurationManager.setKnob(1, "userProvidedDesiredVelocityX", scs, -0.4, 0.4);              
      sliderBoardConfigurationManager.setKnob(2, "userProvidedDesiredVelocityY", scs, -0.5, 0.5);              
      sliderBoardConfigurationManager.setKnob(3, "userProvidedDesiredYawRate", scs, -0.3, 0.3);
      sliderBoardConfigurationManager.setKnob(4, "desiredCoMHeight", scs, 0.0, 1.0);
      sliderBoardConfigurationManager.setKnob(5, "subCircleRadius", scs, 0.0, 0.3);
      sliderBoardConfigurationManager.setKnob(6, "xOffsetFromCenterOfHips", scs, -0.5, 0.5);
      sliderBoardConfigurationManager.setKnob(7, "yOffsetFromCenterOfHips", scs, -0.5, 0.5);
      sliderBoardConfigurationManager.setSliderEnum(8, selectedMode);
      
      sliderBoardConfigurationManager.setSlider(1, "minimumVelocityForFullSkew", scs, 0.0, 1.0);
      sliderBoardConfigurationManager.setSlider(2, "strideLength", scs, 0.0, 1.5);
      sliderBoardConfigurationManager.setSlider(3, "stanceWidth", scs, 0.0, 1.0);
      sliderBoardConfigurationManager.setSlider(4, "maxForwardSkew", scs, 0.0, 0.3);
      sliderBoardConfigurationManager.setSlider(5, "maxLateralSkew", scs, 0.0, 0.5);
      sliderBoardConfigurationManager.setSlider(6, "maxYawPerStep", scs, 0.0, Math.PI/2.0);
      sliderBoardConfigurationManager.setSlider(7, "minimumDistanceFromSameSideFoot", scs, 0.0, 0.25);
      sliderBoardConfigurationManager.saveConfiguration(positionCrawlFootstepChooser);
      
      //orientation tuning
      sliderBoardConfigurationManager.setKnob(1, "userProvidedDesiredVelocityX", scs, -0.4, 0.4);
      sliderBoardConfigurationManager.setKnob(2, "userProvidedDesiredVelocityY", scs, -0.5, 0.5);
      sliderBoardConfigurationManager.setKnob(4, "desiredCoMHeight", scs, 0.0, 1.0);
      sliderBoardConfigurationManager.setSliderEnum(8, selectedMode);
      
      sliderBoardConfigurationManager.setSlider(1, "userProvidedDesiredYawRate", scs, -0.3, 0.3);
      sliderBoardConfigurationManager.setSlider(2, "desiredCoMOrientationPitch", scs, -0.5, 0.5);
      sliderBoardConfigurationManager.setSlider(3, "desiredCoMOrientationRoll", scs, -0.5, 0.5);
      sliderBoardConfigurationManager.setSlider(4, "filteredDesiredCoMYawAlphaBreakFrequency", scs, 0.0, 1.0);
      sliderBoardConfigurationManager.setSlider(5, "filteredDesiredCoMPitchAlphaBreakFrequency", scs, 0.0, 1.5);
      sliderBoardConfigurationManager.setSlider(6, "filteredDesiredCoMRollAlphaBreakFrequency", scs, 0.0, 1.0);
      
      sliderBoardConfigurationManager.saveConfiguration(positionCrawlOrientationTuning);
   }
   
   public String getDefaultConfiguration()
   {
      return positionCrawlCoMShift;
   }
}
