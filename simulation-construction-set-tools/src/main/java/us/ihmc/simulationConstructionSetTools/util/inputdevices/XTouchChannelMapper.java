package us.ihmc.simulationConstructionSetTools.util.inputdevices;

public class XTouchChannelMapper implements MidiChannelMapper
{

   @Override
   public int getKnobChannel(int knob)
   {
      if(knob > 32 || knob < 1)
         throw new RuntimeException("Invalid knob");
      
      if(knob <= 16)
      {
         return knob + 9;   
      }
      else
      {
         return knob + 20;
      }
      }

   @Override
   public int getSliderChannel(int slider)
   {
      if(slider > 18 || slider < 1)
         throw new RuntimeException("Invalid slider");
      
      if(slider <= 9)
      {
         return slider;         
      }
      else
      {
         return slider + 18;
      }
   }

   @Override
   public int getButtonChannel(int button)
   {
      if(button > 78 || button < 1)
         throw new RuntimeException("Invalid button");

      if(button <= 39)
      {
         return button + 15 + 127;
      }
      else
      {
         return button + 31 + 127;
      }
   }

   @Override
   public int getKnobButtonChannel(int knob)
   {
      if(knob > 32 || knob < 1)
         throw new RuntimeException("Invalid knob button");
      if(knob <= 16)
      {
         return knob - 1 + 127;
      }
      else
      {
         return knob + 38 + 127;
      }
   }

}
