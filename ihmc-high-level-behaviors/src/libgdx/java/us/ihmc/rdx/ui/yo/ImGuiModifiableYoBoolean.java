package us.ihmc.rdx.ui.yo;

import imgui.type.ImBoolean;
import us.ihmc.behaviors.tools.yo.YoBooleanClientHelper;
import us.ihmc.tools.Timer;

public class ImGuiModifiableYoBoolean
{
   private final YoBooleanClientHelper yoBooleanHelper;
   private final ImBoolean imBoolean = new ImBoolean();
   private final Timer setValueTimer = new Timer();
   private boolean lastSetValue = false;

   public ImGuiModifiableYoBoolean(YoBooleanClientHelper yoBooleanHelper)
   {
      this.yoBooleanHelper = yoBooleanHelper;
   }

   public void update()
   {
      if (!setValueTimer.isRunning(2.0))
      {
         imBoolean.set(yoBooleanHelper.get());
      }
   }

   public void set()
   {
      lastSetValue = imBoolean.get();
      yoBooleanHelper.set(lastSetValue);
      setValueTimer.reset();
   }

   public YoBooleanClientHelper getYoBooleanHelper()
   {
      return yoBooleanHelper;
   }

   public ImBoolean getImBoolean()
   {
      return imBoolean;
   }
}
