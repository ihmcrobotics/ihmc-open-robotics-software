package us.ihmc.rdx.ui.missionControl;

import mission_control_msgs.msg.dds.SystemAvailableMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.rdx.imgui.ImGuiGlfwWindow;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;

public class MissionControlUI
{
   private final Map<String, SystemAvailableMessage> lastSystemAvailableMessage = new HashMap<>();
   private final Map<String, ImGuiMachine> machines = new HashMap<>();

   private final ROS2Node ros2Node;

   public MissionControlUI()
   {
      ImGuiGlfwWindow imGuiGlfwWindow = new ImGuiGlfwWindow(getClass(), "Mission Control 3");
      imGuiGlfwWindow.runWithSinglePanel(this::renderImGuiWidgets);

      ExceptionHandlingThreadScheduler updateMachinesScheduler = new ExceptionHandlingThreadScheduler("UpdateMachinesScheduler");
      updateMachinesScheduler.schedule(this::updateMachines, 1.00);

      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "mission_control_ui");

      ROS2Tools.createCallbackSubscription(ros2Node, ROS2Tools.SYSTEM_AVAILABLE, subscriber ->
      {
         SystemAvailableMessage message = subscriber.takeNextData();
         String machineInstanceId = message.getInstanceIdAsString();
         lastSystemAvailableMessage.put(machineInstanceId, message);
      });
   }

   private void updateMachines()
   {
      long now = System.currentTimeMillis();

      for (Map.Entry<String, SystemAvailableMessage> entry : lastSystemAvailableMessage.entrySet())
      {
         String machineInstanceId = entry.getKey();
         SystemAvailableMessage message = entry.getValue();
         // Consider expired if we haven't received a SystemAvailableMessage within the last 5 seconds
         boolean expired = (now - message.getEpochTime()) > TimeUnit.SECONDS.toMillis(5);

         // Check for new machines
         if (!expired && !machines.containsKey(machineInstanceId))
         {
            ImGuiMachine machine = new ImGuiMachine(machineInstanceId, message.getHostnameAsString());
            machines.put(machineInstanceId, machine);
         }
         else if (expired)
         {
            machines.remove(machineInstanceId);
         }
      }
   }

   private void renderImGuiWidgets()
   {
      for (ImGuiMachine machine : machines.values())
         machine.renderImGuiWidgets();
   }

   public static void main(String[] args)
   {
      new MissionControlUI();
   }
}
