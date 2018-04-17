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

      for (int slider = 1; slider < 9; slider++)
      {
         int sliderIndex = slider;
         sliderboard.addListener(value -> PrintTools.info("Slider " + sliderIndex + " moved to: " + value), slider);
      }

      Random random = new Random();
      for (int i = 0; i < 10; i++)
      {
         int sliderIndex = random.nextInt(8) + 1;
         sliderboard.setSliderValue(random.nextDouble(), sliderIndex);
         ThreadTools.sleep(100);
      }

      sliderboard.close();
   }
}
