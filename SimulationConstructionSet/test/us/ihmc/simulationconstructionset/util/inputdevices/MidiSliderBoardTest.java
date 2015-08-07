package us.ihmc.simulationconstructionset.util.inputdevices;


import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;


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

	@EstimatedDuration
	@Test(timeout=300000)
   public void testWeirdCase()
   {
      MidiSliderBoard midiSliderBoard = new MidiSliderBoard(null, false);
      YoVariableRegistry registry = new YoVariableRegistry("testRegistry");
      DoubleYoVariable yoVariable = new DoubleYoVariable("test", registry);
      midiSliderBoard.setSlider(3, yoVariable, 1.5, 2.5);
   }

}
