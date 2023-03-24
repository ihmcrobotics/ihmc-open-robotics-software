package us.ihmc.rdx.ui.missionControl;

import imgui.ImGui;
import mission_control_msgs.msg.dds.SystemAvailableMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.rdx.imgui.ImGuiGlfwWindow;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;

import java.util.HashMap;
import java.util.Map;
import java.util.UUID;
import java.util.concurrent.TimeUnit;

public class MissionControlUI
{
   private final Map<UUID, SystemAvailableMessage> lastSystemAvailableMessage = new HashMap<>();
   private final Map<UUID, ImGuiMachine> machines = new HashMap<>();
   private final ROS2Node ros2Node;

   private static ImGuiGlfwWindow window;

   public static ImGuiGlfwWindow getWindow()
   {
      return window;
   }

   public MissionControlUI()
   {
      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "mission_control_ui");

      ROS2Tools.createCallbackSubscription(ros2Node, ROS2Tools.SYSTEM_AVAILABLE, subscriber ->
      {
         SystemAvailableMessage message = subscriber.takeNextData();
         String instanceIdString = message.getInstanceIdAsString();
         UUID instanceId;
         try
         {
            instanceId = UUID.fromString(instanceIdString);
         }
         catch (IllegalArgumentException e)
         {
            LogTools.error("Unable to create instanceId from " + instanceIdString);
            return;
         }
         lastSystemAvailableMessage.put(instanceId, message);
      });

      window = new ImGuiGlfwWindow(getClass(), "Mission Control 3");

      ImGuiPanel controlPanel = new ImGuiPanel("Control Panel", this::renderImGuiWidgets);

      window.getImGuiDockSystem().getPanelManager().addPanel(controlPanel);

      ThreadTools.startAThread(() -> window.run(null, () ->
      {
      }, () -> System.exit(0)), getClass().getName());

      ExceptionHandlingThreadScheduler updateMachinesScheduler = new ExceptionHandlingThreadScheduler("UpdateMachinesScheduler");
      updateMachinesScheduler.schedule(this::updateMachines, 1.0);
   }

   private void updateMachines()
   {
      long now = System.currentTimeMillis();

      for (Map.Entry<UUID, SystemAvailableMessage> entry : lastSystemAvailableMessage.entrySet())
      {
         UUID instanceId = entry.getKey();
         SystemAvailableMessage message = entry.getValue();
         // Consider expired if we haven't received a SystemAvailableMessage within the last 5 seconds
         boolean expired = (now - message.getEpochTime()) > TimeUnit.SECONDS.toMillis(5);

         // Check for new machines
         if (!expired && !machines.containsKey(instanceId))
         {
            ImGuiMachine machine = new ImGuiMachine(instanceId, message.getHostnameAsString(), ros2Node);
            machines.put(instanceId, machine);
         }
         else if (expired)
         {
            machines.remove(instanceId);
         }
      }
   }

   public void renderImGuiWidgets()
   {
      boolean hadMachine = false;

      for (Map.Entry<UUID, ImGuiMachine> entry : machines.entrySet())
      {
         UUID instanceId = entry.getKey();
         ImGuiMachine machine = entry.getValue();

         machine.renderImGuiWidgets();
         ImGui.newLine();

         hadMachine = true;
      }

      if (!hadMachine)
      {
         ImGui.pushFont(ImGuiTools.getMediumFont());
         ImGui.text("No machines");
         ImGui.popFont();
      }
   }

   public static void main(String[] args)
   {
      new MissionControlUI();
   }
}
