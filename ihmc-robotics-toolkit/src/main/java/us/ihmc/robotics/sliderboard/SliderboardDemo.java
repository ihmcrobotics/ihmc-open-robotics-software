package us.ihmc.robotics.sliderboard;

import java.util.Random;

import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;

public class SliderboardDemo
{
   public static void main(String[] args)
   {
      Sliderboard sliderboard = new Sliderboard();

      if (!sliderboard.isConnected())
      {
         return;
      }

      for (int slider = 1; slider <= 8; slider++)
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

      for (int button = 1; button <= 16; button++)
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

      Random random = new Random();
      for (int i = 0; i < 20; i++)
      {
         int sliderIndex = random.nextInt(8) + 1;
         sliderboard.setSliderValue(random.nextDouble(), sliderIndex);
         ThreadTools.sleep(100);
      }
      for (int i = 0; i < 20; i++)
      {
         int buttonIndex = random.nextInt(16) + 1;
         sliderboard.setButtonValue(random.nextBoolean(), buttonIndex);
         ThreadTools.sleep(100);
      }

      sliderboard.close();
   }
}
