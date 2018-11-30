package us.ihmc.simulationConstructionSetTools.util.inputdevices;


import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

@ContinuousIntegrationPlan(categories = IntegrationCategory.UI)
public class XTouchCompactMidiSliderBoardTest
{
   @Before
   public void setUp() throws Exception
   {
   }

   @After
   public void tearDown() throws Exception
   {
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testWeirdCase() throws InterruptedException
   {
      MidiSliderBoard midiSliderBoard = new MidiSliderBoard(null, true);
      YoVariableRegistry registry = new YoVariableRegistry("testRegistry");

      // Sliders
      for(int s = 1; s <=18; s++ )
      {
         YoDouble yoVariable = new YoDouble("slider" + s, registry);
         midiSliderBoard.setSlider(s, yoVariable, 1.5, 2.5); //set scale
         yoVariable.addVariableChangedListener(new VariableChangedListener()
         {
            @Override public void notifyOfVariableChange(YoVariable<?> v)
            {
               System.out.println("Slider changed value: " + v.getValueAsDouble());
            }
         });
      }
      
      // Knobs
      for(int k = 1; k <= 32; k++)
      {
         YoDouble yoVariable = new YoDouble("knob" + k, registry);
         midiSliderBoard.setKnob(k, yoVariable, 1.5, 2.5); //set scale
         yoVariable.addVariableChangedListener(new VariableChangedListener()
         {
            @Override public void notifyOfVariableChange(YoVariable<?> v)
            {
               System.out.println("Knob changed value: " + v.getValueAsDouble());
            }
         });
      }
      
      // Buttons
      for(int b = 1; b <= 78; b++)
      {
         YoBoolean yoVariable = new YoBoolean("button" + b, registry);
         midiSliderBoard.setButton(b, yoVariable); //set scale
         yoVariable.addVariableChangedListener(new VariableChangedListener()
         {
            @Override public void notifyOfVariableChange(YoVariable<?> v)
            {
               System.out.println("Button changed value: " + v.getValueAsDouble());
            }
         });

      }
      // Knob Buttons
      for(int b = 1; b <= 32; b++)
      {
         YoBoolean yoVariable = new YoBoolean("knobButton" + b, registry);
         midiSliderBoard.setKnobButton(b, yoVariable); //set scale
         yoVariable.addVariableChangedListener(new VariableChangedListener()
         {
            @Override public void notifyOfVariableChange(YoVariable<?> v)
            {
               System.out.println("Knob button changed value: " + v.getValueAsDouble());
            }
         });
         
      }
      
      
      registry.getVariable("slider3").setValueFromDouble(2.0);
      registry.getVariable("knob3").setValueFromDouble(2.0);
      registry.getVariable("button2").setValueFromDouble(1.0);
      
      Thread.currentThread().join();
   }

}
