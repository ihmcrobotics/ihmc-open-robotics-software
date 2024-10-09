package us.ihmc.parameterTuner.sliderboard;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;

public class SliderboardDemo
{
   private static final int buttons = 16;
   private static final int sliders = 16;

   public static void main(String[] args)
   {
      Sliderboard sliderboard = new Sliderboard();

      if (!sliderboard.isConnected())
      {
         return;
      }

      for (int slider = 1; slider <= sliders; slider++)
      {
         int sliderIndex = slider;

         sliderboard.addListener(new SliderboardListener()
         {
            @Override
            public void sliderMoved(double sliderPercentage)
            {
               PrintTools.info("Slider " + sliderIndex + " moved to: " + sliderPercentage);
            }
         }, slider);

         sliderboard.setSliderValue(0.0, sliderIndex);
      }

      for (int button = 1; button <= buttons; button++)
      {
         int buttonIndex = button;

         sliderboard.addListener(new ButtonListener()
         {
            @Override
            public void buttonPressed(boolean status)
            {
               PrintTools.info("Button " + buttonIndex + " pushed: " + status);
            }
         }, button);
      }

      goToZero(sliderboard);

      double time = 0.0;
      double dt = 0.05;
      double totalTime = 5.0;
      int button = 1;

      while (time < totalTime)
      {
         for (int slider = 1 ; slider <= sliders; slider++)
         {
            double phase = Math.PI * slider / sliders + Math.PI;
            sliderboard.setSliderValue(Math.max(Math.sin(time * 2.0 * Math.PI / totalTime + phase), 0.0), slider);
         }

         sliderboard.setButtonValue(false, (button - 1) == 0 ? buttons : button - 1);
         sliderboard.setButtonValue(true, button);
         button = button == buttons ? 1 : button + 1;

         ThreadTools.sleep((int) Conversions.secondsToMilliseconds(dt));
         time += dt;
      }

      goToZero(sliderboard);
      sliderboard.close();
   }

   private static void goToZero(Sliderboard sliderboard)
   {
      for (int button = 1 ; button <= buttons; button++)
      {
         sliderboard.setButtonValue(false, button);
      }
      for (int slider = 1 ; slider <= sliders; slider++)
      {
         sliderboard.setSliderValue(0.0, slider);
      }
      ThreadTools.sleep(100);
   }
}
