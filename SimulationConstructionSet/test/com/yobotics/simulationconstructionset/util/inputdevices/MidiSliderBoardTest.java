package com.yobotics.simulationconstructionset.util.inputdevices;


import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.yoUtilities.YoVariableRegistry;

import com.yobotics.simulationconstructionset.DoubleYoVariable;

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

   @Test
   public void testWeirdCase()
   {
      MidiSliderBoard midiSliderBoard = new MidiSliderBoard(null, false);
      YoVariableRegistry registry = new YoVariableRegistry("testRegistry");
      DoubleYoVariable yoVariable = new DoubleYoVariable("test", registry);
      midiSliderBoard.setSlider(3, yoVariable, 1.5, 2.5);
   }

}
