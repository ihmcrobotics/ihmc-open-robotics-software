package us.ihmc.rdx.vr;

public class ClickTools
{
   // TODO: Implement double click checker

   // delay setting
   public final static double CLICK_DELAY_TIME = 300; // ms
   private long delayStartTime = 0;

   public void delay()
   {
      delayStartTime = System.nanoTime();
   }

   public boolean resume()
   {
      long currentTime = System.nanoTime();
      double elapsedTime = (currentTime - delayStartTime) / 1e6;
      return elapsedTime > CLICK_DELAY_TIME;
   }
}
