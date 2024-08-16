package us.ihmc.rdx.imgui;

import us.ihmc.tools.Timer;

public class ImGuiFlashingColors
{
   public static final double DEFAULT_PERIOD = 1.0;

   private final double period;
   private final Timer timer = new Timer();
   protected boolean flashState = false;
   protected final int flashColor1;
   protected final int flashColor2;

   public ImGuiFlashingColors(int flashColor1, int flashColor2)
   {
      this(DEFAULT_PERIOD, flashColor1, flashColor2);
   }

   public ImGuiFlashingColors(double period, int flashColor1, int flashColor2)
   {
      this.period = period;
      this.flashColor1 = flashColor1;
      this.flashColor2 = flashColor2;
   }

   public int getColor(boolean flashing)
   {
      update();

      return flashing && flashState ? flashColor1 : flashColor2;
   }

   protected void update()
   {
      if (!timer.isRunning(period))
      {
         timer.reset();
         flashState = !flashState;
      }
   }
}
