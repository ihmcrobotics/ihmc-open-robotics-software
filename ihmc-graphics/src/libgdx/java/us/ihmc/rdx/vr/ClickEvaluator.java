package us.ihmc.rdx.vr;

public class ClickEvaluator
{
   // Click refers to where a user pushes a button and lets go.
   // Down refers to where a user pushes a button and the button stays down

   // click
   private long bTimeClickedNanosPrevious;
   private long bTimeClickedNanos = 0;
   private static double DOUBLE_CLICK_TIME_THRESHOLD; // ms
   private double clickIntervalMillisecond = Double.POSITIVE_INFINITY;

   // pressed down
   private long bDownStart = -1; // -1 value assigned before press starts
   private long bDownEnd = 0;
   private double bDownDuration = Double.NEGATIVE_INFINITY;

   public enum Click
   {
      NONE, SINGLE, DOUBLE
   }

   public ClickEvaluator()
   {
      // Default double click speed
      DOUBLE_CLICK_TIME_THRESHOLD = 500; // 500 ms
   }

   public ClickEvaluator(double doubleClickTimeThreshold)
   {
      // user set double click speed
      DOUBLE_CLICK_TIME_THRESHOLD = doubleClickTimeThreshold;
   }

   public void registerClick(boolean buttonClicked)
   {
      if (buttonClicked)
      {
         bTimeClickedNanosPrevious = bTimeClickedNanos;
         bTimeClickedNanos = System.nanoTime();
         clickIntervalMillisecond = (bTimeClickedNanos - bTimeClickedNanosPrevious) / 1e6;
      }
   }

   private void resetSingle()
   {
      bDownDuration = Double.NEGATIVE_INFINITY;
      bDownStart = -1;
      bDownEnd = -1;
   }

   private void resetDouble()
   {
      clickIntervalMillisecond = Double.POSITIVE_INFINITY;
   }

   public boolean isDoubleClick()
   {
      return clickIntervalMillisecond < DOUBLE_CLICK_TIME_THRESHOLD;
   }

   public void registerPressedDown(boolean isButtonDown)
   {
      if (!isButtonDown)
      {
         bDownStart = -1;
      }
      if (isButtonDown && bDownStart == -1)
      {
         // Start of pressed down moment
         bDownStart = System.nanoTime();
      }
      // button down
      if (bDownStart != -1)
      {
         // register end of pressed down state
         bDownEnd = System.nanoTime();
         // save duration of time pressed
         bDownDuration = bDownEnd - bDownStart;
      }
   }

   private boolean isSingleClick()
   {
      return bDownDuration > DOUBLE_CLICK_TIME_THRESHOLD;
   }

   public Click evaluateClick(boolean bClicked, boolean bPressedDown)
   {
      registerClick(bClicked);
      registerPressedDown(bPressedDown);

      if (isDoubleClick())
      {
         resetDouble();
         return Click.DOUBLE;
      }
      else if (isSingleClick())
      {
         resetSingle();
         return Click.SINGLE;
      }
      else
      {
         return Click.NONE;
      }
   }
}
