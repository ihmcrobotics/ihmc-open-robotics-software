package us.ihmc.gdx.simulation;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.DoorLocationPacket;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.imgui.ImGuiLabelMap;
import us.ihmc.gdx.simulation.environment.object.objects.GDXDoorOnlyObject;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.affordances.GDXPoseModifiableObject;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.tools.thread.PausablePeriodicThread;

public class GDXDoorSimulator
{
   private final CommunicationHelper helper;
   private final ImGuiLabelMap labels = new ImGuiLabelMap();
   private final String windowName = labels.get("Door Simulator");
   private final ImBoolean enabled = new ImBoolean(false);
   private final PausablePeriodicThread pausablePeriodicThread;

   private final GDXPoseModifiableObject modifiableDoor = new GDXPoseModifiableObject();
   private final RigidBodyTransform tempDoorTransform = new RigidBodyTransform();

   public GDXDoorSimulator(RemoteSyncedRobotModel syncedRobot, CommunicationHelper helper)
   {
      this.helper = helper;
      pausablePeriodicThread = new PausablePeriodicThread("DoorSimulator", 1.0 / 5.0, this::update);

      // subscribe to robot hand frame

      // animate door yaw when hand touches
   }

   private void update()
   {
      GDXTools.toEuclid(modifiableDoor.getObject().getRealisticModelInstance().transform, tempDoorTransform);
      helper.publish(ROS2Tools::getDoorLocationTopic, HumanoidMessageTools.createDoorLocationPacket(tempDoorTransform,
                                                                                                    DoorLocationPacket.PUSH_HANDLE_RIGHT));
   }

   public void create(GDXImGuiBasedUI baseUI)
   {
      modifiableDoor.create(baseUI, new GDXDoorOnlyObject());
   }

   public void renderImGuiWindow()
   {
      ImGui.begin(windowName);
      if (ImGui.checkbox(labels.get("Enabled"), enabled))
      {
         pausablePeriodicThread.setRunning(enabled.get());
      }
      ImGui.end();
   }

   public void setEnabled(boolean enabled)
   {
      this.enabled.set(enabled);
      pausablePeriodicThread.setRunning(enabled);
   }

   public void getRealRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (enabled.get())
         modifiableDoor.getRealRenderables(renderables, pool);
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (enabled.get())
         modifiableDoor.getVirtualRenderables(renderables, pool);
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
