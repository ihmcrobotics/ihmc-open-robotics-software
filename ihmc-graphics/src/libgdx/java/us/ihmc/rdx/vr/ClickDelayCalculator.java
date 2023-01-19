package us.ihmc.rdx.vr;

import us.ihmc.tools.Timer;

/**
 *  This class can be used whenever one needs to implement double-click for controllers and avoid triggering single-click by adding a delay after each click
 *     Default double click time interval we use is 250 ms, so we give each click a 300 ms window to wait.
 */
public class ClickDelayCalculator
{
   // delay setting
   public final static double CLICK_DELAY_TIME = 300; // ms
   private final Timer timer = new Timer();

   public void delay()
   {
      timer.reset();
   }

   public boolean resume()
   {
      return timer.getElapsedTime() > CLICK_DELAY_TIME;
   }
}
