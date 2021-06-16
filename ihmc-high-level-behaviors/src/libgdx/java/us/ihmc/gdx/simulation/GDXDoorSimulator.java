package us.ihmc.gdx.simulation;

import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.gdx.imgui.ImGuiLabelMap;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.objects.GDXPushHandleRightDoorObject;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.tools.thread.PausablePeriodicThread;

public class GDXDoorSimulator
{
   private final CommunicationHelper helper;
   private final ImGuiLabelMap labels = new ImGuiLabelMap();
   private final String windowName = labels.get("Door Simulator");
   private final ImBoolean enabled = new ImBoolean(false);
   private final PausablePeriodicThread pausablePeriodicThread;

   private GDXEnvironmentObject door;

   public GDXDoorSimulator(ROS2SyncedRobotModel syncedRobot, CommunicationHelper helper)
   {
      this.helper = helper;
      pausablePeriodicThread = new PausablePeriodicThread("DoorSimulator", 1.0 / 5.0, this::update);

      // subscribe to robot hand frame

      // animate door yaw when hand touches
   }

   // TODO: Maybe accept id with door and support multiple doors
   public void setDoor(GDXPushHandleRightDoorObject door)
   {
      this.door = door;
   }

   private void update()
   {
      if (door != null)
      {
//         helper.publish(ROS2Tools::getDoorLocationTopic,
//                        HumanoidMessageTools.createDoorLocationPacket(new RigidBodyTransform(door.getObjectTransform()), DoorLocationPacket.PUSH_HANDLE_RIGHT));
      }
   }

   public void create(GDXImGuiBasedUI baseUI)
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
