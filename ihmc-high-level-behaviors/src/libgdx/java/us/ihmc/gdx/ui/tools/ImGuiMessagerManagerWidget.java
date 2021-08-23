package us.ihmc.gdx.ui.tools;

import imgui.internal.ImGui;
import us.ihmc.behaviors.BehaviorModule;
import us.ihmc.behaviors.tools.MessagerHelper;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.messager.SharedMemoryMessager;

import java.util.function.Supplier;

public class ImGuiMessagerManagerWidget
{
   private volatile boolean messagerConnecting = false;
   private String messagerConnectedHost = "";
   private final MessagerHelper messagerHelper;
   private final Supplier<String> hostSupplier;

   public ImGuiMessagerManagerWidget(MessagerHelper messagerHelper, Supplier<String> hostSupplier)
   {
      this.messagerHelper = messagerHelper;
      this.hostSupplier = hostSupplier;
   }

   public void renderImGuiWidgets()
   {
      if (messagerConnecting)
      {
         ImGui.text("Messager connecting...");
         if (messagerHelper.isConnected())
         {
            messagerConnecting = false;
         }
      }
      else if (messagerHelper.isDisconnecting())
      {
         ImGui.text("Messager disconnecting...");
      }
      else if (!messagerHelper.isConnected())
      {
         if (ImGui.button("Connect messager")) // TODO: One button should connect both
         {
            connectViaKryo(hostSupplier.get());
         }

         SharedMemoryMessager potentialSharedMemoryMessager = BehaviorModule.getSharedMemoryMessager();
         if (potentialSharedMemoryMessager != null && potentialSharedMemoryMessager.isMessagerOpen())
         {
            if (ImGui.button("Use shared memory messager"))
            {
               messagerHelper.connectViaSharedMemory(potentialSharedMemoryMessager);
            }
         }
      }
      else
      {
         if (messagerHelper.isUsingSharedMemory())
         {
            ImGui.text("Using shared memory messager.");
         }
         else
         {
            ImGui.text("Messager connected to " + messagerConnectedHost + ".");
         }
         ImGui.sameLine();
         if (ImGui.button(ImGuiTools.uniqueLabel(this, "Disconnect")))
         {
            disconnectMessager();
         }
      }
   }

   public void connectViaKryo(String hostname)
   {
      messagerHelper.connectViaKryo(hostSupplier.get(), NetworkPorts.BEHAVIOR_MODULE_MESSAGER_PORT.getPort());
      messagerConnectedHost = hostSupplier.get();
      messagerConnecting = true;
   }

   public void disconnectMessager()
   {
      messagerHelper.disconnect();
   }
}
