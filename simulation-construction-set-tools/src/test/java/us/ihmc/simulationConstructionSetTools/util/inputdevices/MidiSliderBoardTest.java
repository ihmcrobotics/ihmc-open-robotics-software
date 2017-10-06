package us.ihmc.simulationConstructionSetTools.util.inputdevices;


import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

@ContinuousIntegrationPlan(categories = IntegrationCategory.UI)
public class MidiSliderBoardTest
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
   public void testWeirdCase()
   {
      MidiSliderBoard midiSliderBoard = new MidiSliderBoard(null, true);
      YoVariableRegistry registry = new YoVariableRegistry("testRegistry");

      //need one of these for each DOF
      YoDouble yoVariable = new YoDouble("test", registry);
      midiSliderBoard.setSlider(3, yoVariable, 1.5, 2.5); //set scale
      yoVariable.addVariableChangedListener(new VariableChangedListener()
      {
         @Override public void notifyOfVariableChange(YoVariable<?> v)
         {
            System.out.println(v.getValueAsDouble());
         }
      });
      while(true)
      {

      }
   }

}
