package us.ihmc.rdx.ui.yo;

import imgui.type.ImDouble;
import us.ihmc.behaviors.tools.yo.YoDoubleClientHelper;
import us.ihmc.tools.Timer;

public class ImGuiModifiableYoDouble
{
   private final YoDoubleClientHelper yoDoubleHelper;
   private final ImDouble imDouble = new ImDouble();
   private final Timer setValueTimer = new Timer();
   private double lastSetValue = Double.NaN; // TODO: Do something smart here?

   public ImGuiModifiableYoDouble(YoDoubleClientHelper yoDoubleHelper)
   {
      this.yoDoubleHelper = yoDoubleHelper;
   }

   public void update()
   {
      if (!setValueTimer.isRunning(2.0))
      {
         imDouble.set(yoDoubleHelper.get());
      }
   }

   public void set()
   {
      lastSetValue = imDouble.get();
      yoDoubleHelper.set(lastSetValue);
      setValueTimer.reset();
   }

   public YoDoubleClientHelper getYoDoubleHelper()
   {
      return yoDoubleHelper;
   }

   public ImDouble getImDouble()
   {
      return imDouble;
   }
}
