package us.ihmc.simulationconstructionset.util.inputdevices;


import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;


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
      DoubleYoVariable yoVariable = new DoubleYoVariable("test", registry);
      midiSliderBoard.setSlider(3, yoVariable, 1.5, 2.5); //set scale
      yoVariable.addVariableChangedListener(new VariableChangedListener()
      {
         @Override public void variableChanged(YoVariable<?> v)
         {
            System.out.println(v.getValueAsDouble());
         }
      });
      while(true)
      {

      }
   }

}
