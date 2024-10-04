package us.ihmc.rdx.vr;

/**
 * This class can be used whenever one needs to implement double-click for controllers and avoid triggering single-click by adding a delay after each click.
 * Default double click time interval we use is 250 ms, so we give each click a 300 ms window to wait.
 */
public class ClickDelayCalculator
{
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
