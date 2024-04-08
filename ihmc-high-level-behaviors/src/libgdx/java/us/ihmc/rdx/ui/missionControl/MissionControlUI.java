package us.ihmc.rdx.ui.missionControl;

import imgui.ImGui;
import mission_control_msgs.msg.dds.SystemAvailableMessage;
import us.ihmc.communication.MissionControlAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.rdx.imgui.ImGuiGlfwWindow;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;

import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.TimeUnit;

public class MissionControlUI
{
   private final Map<UUID, Long> lastSystemAvailableMessageTimes = new HashMap<>();
   private final Map<UUID, SystemAvailableMessage> lastSystemAvailableMessage = new HashMap<>();
   private final Map<UUID, ImGuiMachine> machines = new ConcurrentHashMap<>(); // Accessed by several threads
   private final ROS2Node ros2Node;

   private final ImGuiGlfwWindow window;

   // Render loop helpers
   private final Map<UUID, Long> rebootConfirmations = new HashMap<>();
   private final Set<UUID> rebootWaiting = new HashSet<>();

   public MissionControlUI()
   {
      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "mission_control_ui");

      ros2Node.createSubscription(MissionControlAPI.SYSTEM_AVAILABLE, subscriber ->
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
         lastSystemAvailableMessageTimes.put(instanceId, System.currentTimeMillis());
         lastSystemAvailableMessage.put(instanceId, message);
      });

      window = new ImGuiGlfwWindow(getClass(), "Mission Control 3");

      window.setIcons("icons/mission-control-3-icon16.png", "icons/mission-control-3-icon32.png", "icons/mission-control-3-icon48.png");

      ExceptionHandlingThreadScheduler updateMachinesScheduler = new ExceptionHandlingThreadScheduler("UpdateMachinesScheduler");
      updateMachinesScheduler.schedule(this::updateMachines, 1.0);

      // Blocking
      window.runWithSinglePanel(this::renderImGuiWidgets);
   }

   private void updateMachines()
   {
      long now = System.currentTimeMillis();

      for (Map.Entry<UUID, SystemAvailableMessage> entry : lastSystemAvailableMessage.entrySet())
      {
         UUID instanceId = entry.getKey();
         SystemAvailableMessage message = entry.getValue();
         long time = lastSystemAvailableMessageTimes.get(instanceId);
         // Consider expired if we haven't received a SystemAvailableMessage within the last 5 seconds
         boolean expired = (now - time) > TimeUnit.SECONDS.toMillis(5);

         // Check for new machines
         if (!expired && !machines.containsKey(instanceId))
         {
            ImGuiMachine machine = new ImGuiMachine(instanceId, message.getHostnameAsString(), ros2Node);
            window.getPanelManager().queueAddPanel(machine.getPanel());
            machines.put(instanceId, machine);
         }
         else if (expired)
         {
            ImGuiMachine machine = machines.remove(instanceId);

            if (machine != null)
            {
               window.getPanelManager().queueRemovePanel(machine.getPanel());
            }
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

         ImGui.pushFont(ImGuiTools.getMediumFont());
         ImGui.text(machine.getHostname());
         ImGui.popFont();

         String uptime = machine.getLastResourceUsageMessage().getUptime().toString().trim();
         ImGui.text(uptime);

         if (rebootConfirmations.containsKey(instanceId))
         {
            // Remove from reboot confirmations after 5 seconds
            if (System.currentTimeMillis() - rebootConfirmations.get(instanceId) > 5000)
            {
               rebootConfirmations.remove(instanceId);
            }

            if (ImGui.button("Reboot (confirm)"))
            {
               rebootConfirmations.remove(instanceId);
               rebootWaiting.add(instanceId);
               machine.sendRebootRequest();
            }
         }
         else if (rebootWaiting.contains(instanceId))
         {
            ImGui.beginDisabled(true);
            ImGui.button("Rebooting");
            ImGui.endDisabled();
         }
         else
         {
            if (ImGui.button("Reboot"))
            {
               rebootConfirmations.put(instanceId, System.currentTimeMillis());
            }
         }

         // Add left padding
         ImGui.setCursorPos(ImGui.getCursorPosX() + 20f, ImGui.getCursorPosY());
         {
            ImGui.beginGroup();
            machine.renderImGuiWidgets();
            ImGui.newLine();
            ImGui.endGroup();
         }

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
