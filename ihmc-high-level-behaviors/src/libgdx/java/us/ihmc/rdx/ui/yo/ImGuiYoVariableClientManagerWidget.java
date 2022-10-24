package us.ihmc.rdx.ui.yo;

import imgui.internal.ImGui;
import us.ihmc.behaviors.tools.yo.YoVariableClientHelper;

import java.util.function.Supplier;

public class ImGuiYoVariableClientManagerWidget
{
   private final YoVariableClientHelper yoVariableClientHelper;
   private final Supplier<String> hostSupplier;
   private final int port;

   public ImGuiYoVariableClientManagerWidget(YoVariableClientHelper yoVariableClientHelper, Supplier<String> hostSupplier, int port)
   {
      this.yoVariableClientHelper = yoVariableClientHelper;
      this.hostSupplier = hostSupplier;
      this.port = port;
   }

   public void renderImGuiWidgets()
   {
      if (yoVariableClientHelper.isConnecting())
      {
         ImGui.text("YoVariable client connecting...");
      }
      else if (!yoVariableClientHelper.isConnected())
      {
         if (ImGui.button("Connect YoVariable client"))
         {
            startYoVariableClient();
         }
      }
      else
      {
         ImGui.text("YoVariable client connected to: " + yoVariableClientHelper.getServerName());

         if (ImGui.button("Disconnect YoVariable client"))
         {
            yoVariableClientHelper.disconnect();
         }
      }
   }

   public void startYoVariableClient()
   {
      yoVariableClientHelper.start(hostSupplier.get(), port);
   }

   public void destroy()
   {
      yoVariableClientHelper.disconnect();
   }
}
