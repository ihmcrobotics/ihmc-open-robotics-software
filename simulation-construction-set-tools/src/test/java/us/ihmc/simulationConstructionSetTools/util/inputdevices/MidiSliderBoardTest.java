package us.ihmc.simulationConstructionSetTools.util.inputdevices;


import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

@Tag("gui-slow")
@Disabled
public class MidiSliderBoardTest
{
   @BeforeEach
   public void setUp() throws Exception
   {
   }

   @AfterEach
   public void tearDown() throws Exception
   {
   }

	@Test
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
