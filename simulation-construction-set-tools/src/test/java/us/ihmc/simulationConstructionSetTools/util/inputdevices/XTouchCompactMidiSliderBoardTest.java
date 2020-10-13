package us.ihmc.simulationConstructionSetTools.util.inputdevices;


import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

@Tag("gui-slow")
@Disabled
public class XTouchCompactMidiSliderBoardTest
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
   public void testWeirdCase() throws InterruptedException
   {
      MidiSliderBoard midiSliderBoard = new MidiSliderBoard(null, true);
      YoRegistry registry = new YoRegistry("testRegistry");

      // Sliders
      for(int s = 1; s <=18; s++ )
      {
         YoDouble yoVariable = new YoDouble("slider" + s, registry);
         midiSliderBoard.setSlider(s, yoVariable, 1.5, 2.5); //set scale
         yoVariable.addListener(new YoVariableChangedListener()
         {
            @Override public void changed(YoVariable v)
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
         yoVariable.addListener(new YoVariableChangedListener()
         {
            @Override public void changed(YoVariable v)
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
         yoVariable.addListener(new YoVariableChangedListener()
         {
            @Override public void changed(YoVariable v)
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
         yoVariable.addListener(new YoVariableChangedListener()
         {
            @Override public void changed(YoVariable v)
            {
               System.out.println("Knob button changed value: " + v.getValueAsDouble());
            }
         });

      }


      registry.findVariable("slider3").setValueFromDouble(2.0);
      registry.findVariable("knob3").setValueFromDouble(2.0);
      registry.findVariable("button2").setValueFromDouble(1.0);

      Thread.currentThread().join();
   }

}
