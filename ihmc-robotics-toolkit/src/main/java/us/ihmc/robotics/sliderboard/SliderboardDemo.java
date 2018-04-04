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

      for (int slider = 0; slider < 8; slider++)
      {
         int sliderIndex = slider;
         sliderboard.addListener(value -> PrintTools.info("Slider " + sliderIndex + " moved to: " + value), slider);
      }

      Random random = new Random();
      for (int i = 0; i < 10; i++)
      {
         sliderboard.setSliderValue(random.nextDouble(), random.nextInt(8));
         ThreadTools.sleep(100);
      }

      ThreadTools.sleep(10000);

      sliderboard.close();
   }
}
