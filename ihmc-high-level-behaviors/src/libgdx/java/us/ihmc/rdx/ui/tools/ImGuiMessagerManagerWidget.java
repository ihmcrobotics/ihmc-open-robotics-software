package us.ihmc.rdx.ui.tools;

import imgui.internal.ImGui;
import us.ihmc.behaviors.tools.MessagerHelper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.messager.SharedMemoryMessager;

import java.util.function.Supplier;

public class ImGuiMessagerManagerWidget
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private volatile boolean messagerConnecting = false;
   private String messagerConnectedHost = "";
   private final MessagerHelper messagerHelper;
   private final Supplier<String> hostSupplier;
   private final int port;

   public ImGuiMessagerManagerWidget(MessagerHelper messagerHelper, Supplier<String> hostSupplier, int port)
   {
      this.messagerHelper = messagerHelper;
      this.hostSupplier = hostSupplier;
      this.port = port;
   }

   public void renderImGuiWidgets()
   {
      if (messagerConnecting)
      {
         ImGui.text("Messager connecting...");
         ImGui.sameLine();
         if (messagerHelper.isConnected() || ImGui.button(labels.get("Cancel")))
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
         ImGui.text("Connect using:");
         ImGui.sameLine();
         if (ImGui.button(labels.get("Kryo")))
         {
            connectKryo();
         }
         if (isSharedMemoryAvailable())
         {
            ImGui.sameLine();
            if (ImGui.button(labels.get("Shared memory")))
            {
               connectSharedMemory();
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
         if (ImGui.button(labels.get("Disconnect")))
         {
            disconnectMessager();
         }
      }
   }

   private boolean isSharedMemoryAvailable()
   {
      SharedMemoryMessager potentialSharedMemoryMessager = null; //BehaviorModule.getSharedMemoryMessager();
      return potentialSharedMemoryMessager != null && potentialSharedMemoryMessager.isMessagerOpen();
   }

   public void connect()
   {
      if (isSharedMemoryAvailable()) // Assumption: it's likely that if it's available that we should be using it
      {
         connectSharedMemory();
      }
      else
      {
         connectKryo();
      }
   }

   private void connectSharedMemory()
   {
//      messagerHelper.connectViaSharedMemory(BehaviorModule.getSharedMemoryMessager());
   }

   private void connectKryo()
   {
      messagerHelper.connectViaKryo(hostSupplier.get(), port);
      messagerConnectedHost = hostSupplier.get();
      messagerConnecting = true;
   }

   public void disconnectMessager()
   {
      messagerHelper.disconnect();
   }
}
