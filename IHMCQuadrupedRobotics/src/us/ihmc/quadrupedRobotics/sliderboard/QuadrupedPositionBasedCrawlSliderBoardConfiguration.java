package us.ihmc.quadrupedRobotics.sliderboard;

import us.ihmc.quadrupedRobotics.controller.state.QuadrupedControllerState;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.inputdevices.EnumDependentSliderBoardMapping;
import us.ihmc.simulationconstructionset.util.inputdevices.SliderBoardConfigurationManager;

public class QuadrupedPositionBasedCrawlSliderBoardConfiguration implements EnumDependentSliderBoardMapping<QuadrupedControllerState>
{
   private final String positionCrawlCoMShift = QuadrupedSliderBoardMode.POSITIONCRAWL_COM_SHIFT.toString();
   private final String positionCrawlFootstepChooser = QuadrupedSliderBoardMode.POSITIONCRAWL_FOOTSTEP_CHOOSER.toString();
   private final String positionCrawlOrientationTuning = QuadrupedSliderBoardMode.POSITIONCRAWL_ORIENTATION_TUNING.toString();
   
   private final String variableNamespace = "root.babyBeastSimple.QuadrupedSimulationController.QuadrupedControllerManager.QuadrupedPositionBasedCrawlController.";
   private final EnumYoVariable<QuadrupedSliderBoardMode> sliderboardMode;
   
   public QuadrupedPositionBasedCrawlSliderBoardConfiguration(final SliderBoardConfigurationManager sliderBoardConfigurationManager, SimulationConstructionSet scs)
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
      
      //CoM Shift tuning
      sliderBoardConfigurationManager.setSlider(1, variableNamespace + "userProvidedDesiredVelocityProvider.userProvidedDesiredVelocityX", scs, -0.4, 0.4);
      sliderBoardConfigurationManager.setSlider(2, variableNamespace + "userProvidedDesiredVelocityProvider.userProvidedDesiredVelocityY", scs, -0.5, 0.5);
      sliderBoardConfigurationManager.setSlider(3, variableNamespace + "userProvidedDesiredYawRateProvider.userProvidedDesiredYawRate", scs, -0.3, 0.3);
      sliderBoardConfigurationManager.setSlider(4, variableNamespace + "desiredCoMHeight", scs, 0.0, 1.0);
      sliderBoardConfigurationManager.setSlider(5, variableNamespace + "filteredDesiredCoMHeightAlphaBreakFrequency", scs, 0.0, 1.0);
      sliderBoardConfigurationManager.setSlider(6, variableNamespace + "subCircleRadius", scs, 0.0, 0.3);
//      sliderBoardConfigurationManager.setSliderEnum(8, selectedMode);
      sliderBoardConfigurationManager.saveConfiguration(positionCrawlCoMShift);
      
      //footstep chooser tuning
      sliderBoardConfigurationManager.setKnob(1, variableNamespace + "userProvidedDesiredVelocityProvider.userProvidedDesiredVelocityX", scs, -0.4, 0.4);              
      sliderBoardConfigurationManager.setKnob(2, variableNamespace + "userProvidedDesiredVelocityProvider.userProvidedDesiredVelocityY", scs, -0.5, 0.5);              
      sliderBoardConfigurationManager.setKnob(3, variableNamespace + "userProvidedDesiredYawRateProvider.userProvidedDesiredYawRate", scs, -0.3, 0.3);
      sliderBoardConfigurationManager.setKnob(4, variableNamespace + "desiredCoMHeight", scs, 0.0, 1.0);
      sliderBoardConfigurationManager.setKnob(5, variableNamespace + "subCircleRadius", scs, 0.0, 0.3);
      sliderBoardConfigurationManager.setKnob(6, variableNamespace + "MidFootZUpSwingTargetGenerator.xOffsetFromCenterOfHips", scs, -0.5, 0.5);
      sliderBoardConfigurationManager.setKnob(7, variableNamespace + "MidFootZUpSwingTargetGenerator.yOffsetFromCenterOfHips", scs, -0.5, 0.5);
//      sliderBoardConfigurationManager.setSliderEnum(8, selectedMode);
      
      sliderBoardConfigurationManager.setSlider(1, variableNamespace + "MidFootZUpSwingTargetGenerator.minimumVelocityForFullSkew", scs, 0.0, 1.0);
      sliderBoardConfigurationManager.setSlider(2, variableNamespace + "MidFootZUpSwingTargetGenerator.strideLength", scs, 0.0, 1.5);
      sliderBoardConfigurationManager.setSlider(3, variableNamespace + "MidFootZUpSwingTargetGenerator.stanceWidth", scs, 0.0, 1.0);
      sliderBoardConfigurationManager.setSlider(4, variableNamespace + "MidFootZUpSwingTargetGenerator.maxForwardSkew", scs, 0.0, 0.3);
      sliderBoardConfigurationManager.setSlider(5, variableNamespace + "MidFootZUpSwingTargetGenerator.maxLateralSkew", scs, 0.0, 0.5);
      sliderBoardConfigurationManager.setSlider(6, variableNamespace + "MidFootZUpSwingTargetGenerator.maxYawPerStep", scs, 0.0, Math.PI/2.0);
      sliderBoardConfigurationManager.setSlider(7, variableNamespace + "MidFootZUpSwingTargetGenerator.minimumDistanceFromSameSideFoot", scs, 0.0, 0.25);
      sliderBoardConfigurationManager.saveConfiguration(positionCrawlFootstepChooser);
      
      //orientation tuning
      sliderBoardConfigurationManager.setKnob(1, variableNamespace + "userProvidedDesiredVelocityProvider.userProvidedDesiredVelocityX", scs, -0.4, 0.4);
      sliderBoardConfigurationManager.setKnob(2, variableNamespace + "userProvidedDesiredVelocityProvider.userProvidedDesiredVelocityY", scs, -0.5, 0.5);
      sliderBoardConfigurationManager.setKnob(4, variableNamespace + "desiredCoMHeight", scs, 0.0, 1.0);
//      sliderBoardConfigurationManager.setSliderEnum(8, selectedMode);
      
      sliderBoardConfigurationManager.setSlider(1, variableNamespace + "userProvidedDesiredYawRateProvider.userProvidedDesiredYawRate", scs, -0.3, 0.3);
      sliderBoardConfigurationManager.setSlider(2, variableNamespace + "desiredCoMOrientationPitch", scs, -0.5, 0.5);
      sliderBoardConfigurationManager.setSlider(3, variableNamespace + "desiredCoMOrientationRoll", scs, -0.5, 0.5);
      sliderBoardConfigurationManager.setSlider(4, variableNamespace + "filteredDesiredCoMYawAlphaBreakFrequency", scs, 0.0, 1.0);
      sliderBoardConfigurationManager.setSlider(5, variableNamespace + "filteredDesiredCoMPitchAlphaBreakFrequency", scs, 0.0, 1.5);
      sliderBoardConfigurationManager.setSlider(6, variableNamespace + "filteredDesiredCoMRollAlphaBreakFrequency", scs, 0.0, 1.0);
      
      sliderBoardConfigurationManager.saveConfiguration(positionCrawlOrientationTuning);
   }

   @Override
   public QuadrupedControllerState getEnum()
   {
      return QuadrupedControllerState.POSITION_CRAWL;
   }

   @Override
   public void activateConfiguration(SliderBoardConfigurationManager sliderBoardConfigurationManager)
   {
      sliderboardMode.set(QuadrupedSliderBoardMode.POSITIONCRAWL_COM_SHIFT);
      sliderBoardConfigurationManager.loadConfiguration(positionCrawlCoMShift);
   }
}
