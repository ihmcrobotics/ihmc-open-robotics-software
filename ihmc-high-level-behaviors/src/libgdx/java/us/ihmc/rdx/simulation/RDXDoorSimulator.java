package us.ihmc.rdx.simulation;

import perception_msgs.msg.dds.DoorLocationPacket;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.imgui.ImGuiLabelMap;
import us.ihmc.rdx.simulation.environment.object.objects.door.RDXDoorObject;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.tools.thread.PausablePeriodicThread;

public class RDXDoorSimulator
{
   private final CommunicationHelper helper;
   private final ImGuiLabelMap labels = new ImGuiLabelMap();
   private final String windowName = labels.get("Door Simulator");
   private final ImBoolean enabled = new ImBoolean(false);
   private final PausablePeriodicThread pausablePeriodicThread;

   private RDXDoorObject door;

   public RDXDoorSimulator(ROS2SyncedRobotModel syncedRobot, CommunicationHelper helper)
   {
      this.helper = helper;
      pausablePeriodicThread = new PausablePeriodicThread("DoorSimulator", 1.0 / 5.0, this::update);

      // subscribe to robot hand frame

      // animate door yaw when hand touches
   }

   // TODO: Maybe accept id with door and support multiple doors
   public void setDoor(RDXDoorObject door)
   {
      this.door = door;
   }

   private void update()
   {
      if (door != null)
      {
         helper.publish(PerceptionAPI::getDoorLocationTopic,
                        HumanoidMessageTools.createDoorLocationPacket(new RigidBodyTransform(door.getObjectTransform()), DoorLocationPacket.PUSH_HANDLE_RIGHT));
      }
   }

   public void create(RDXBaseUI baseUI)
   {
   }

   public void renderImGuiWindow()
   {
      ImGui.begin(windowName);
      renderImGuiWidgets();
      ImGui.end();
   }

   public void renderImGuiWidgets()
   {
      if (door == null)
      {
         ImGui.text("No door in scene.");
         enabled.set(false);
         pausablePeriodicThread.setRunning(false);
      }
      else
      {
         if (ImGui.checkbox(labels.get("Door simulator enabled"), enabled))
         {
            pausablePeriodicThread.setRunning(enabled.get());
         }
      }
   }

   public void setEnabled(boolean enabled)
   {
      this.enabled.set(enabled);
      pausablePeriodicThread.setRunning(enabled);
   }

   public void destroy()
   {
      pausablePeriodicThread.destroy();
   }

   public String getWindowName()
   {
      return windowName;
   }
}
