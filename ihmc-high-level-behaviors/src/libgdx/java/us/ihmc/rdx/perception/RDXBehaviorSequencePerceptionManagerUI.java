package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.perception.BehaviorSequencePerceptionManager;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;

import java.util.Set;

/**
 * Should just display the ghost objects and provide the reference frame library.
 */
public class RDXBehaviorSequencePerceptionManagerUI
{
   private final ImGuiPanel panel = new ImGuiPanel("Perception Manager", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean showGraphics = new ImBoolean(true);
   private final ROS2PublishSubscribeAPI ros2;
   private final RDXPerceptionObjectUpdater pullDoorFrame;
   private final RDXPerceptionObjectUpdater pullDoorPanel;
   private final RDXPerceptionObjectUpdater pushDoorFrame;
   private final RDXPerceptionObjectUpdater pushDoorPanel;

   public RDXBehaviorSequencePerceptionManagerUI(ROS2PublishSubscribeAPI ros2)
   {
      this.ros2 = ros2;

      pullDoorFrame = new RDXPerceptionObjectUpdater(ros2,
                                                     BehaviorSequencePerceptionManager.DETECTED_PULL_DOOR_FRAME,
                                                     "environmentObjects/door/doorFrame/DoorFrame.g3dj",
                                                     String.format("PullDoor%dFrame", BehaviorSequencePerceptionManager.PULL_DOOR_MARKER_ID));
      pullDoorPanel = new RDXPerceptionObjectUpdater(ros2,
                                                     BehaviorSequencePerceptionManager.DETECTED_PULL_DOOR_PANEL,
                                                     "environmentObjects/door/doorPanel/DoorPanel.g3dj",
                                                     String.format("PullDoor%dPanel", BehaviorSequencePerceptionManager.PULL_DOOR_MARKER_ID));
      pushDoorFrame = new RDXPerceptionObjectUpdater(ros2,
                                                     BehaviorSequencePerceptionManager.DETECTED_PUSH_DOOR_FRAME,
                                                     "environmentObjects/door/doorFrame/DoorFrame.g3dj",
                                                     String.format("PushDoor%dFrame", BehaviorSequencePerceptionManager.PUSH_DOOR_MARKER_ID));
      pushDoorPanel = new RDXPerceptionObjectUpdater(ros2,
                                                     BehaviorSequencePerceptionManager.DETECTED_PUSH_DOOR_PANEL,
                                                     "environmentObjects/door/doorPanel/DoorPanel.g3dj",
                                                     String.format("PushDoor%dPanel", BehaviorSequencePerceptionManager.PUSH_DOOR_MARKER_ID));
   }

   public void update()
   {
      pullDoorFrame.update();
      pullDoorPanel.update();
      pushDoorFrame.update();
      pushDoorPanel.update();
   }

   public void renderImGuiWidgets()
   {
      ImGui.checkbox(labels.get("Show graphics"), showGraphics);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (showGraphics.get())
      {
         pullDoorFrame.getObject().getRenderables(renderables, pool, sceneLevels);
         pullDoorPanel.getObject().getRenderables(renderables, pool, sceneLevels);
         pushDoorFrame.getObject().getRenderables(renderables, pool, sceneLevels);
         pushDoorPanel.getObject().getRenderables(renderables, pool, sceneLevels);
      }
   }

   public ImGuiPanel getPanel()
   {
      return panel;
   }

   public RDXPerceptionObjectUpdater getPullDoorFrame()
   {
      return pullDoorFrame;
   }

   public RDXPerceptionObjectUpdater getPullDoorPanel()
   {
      return pullDoorPanel;
   }

   public RDXPerceptionObjectUpdater getPushDoorFrame()
   {
      return pushDoorFrame;
   }

   public RDXPerceptionObjectUpdater getPushDoorPanel()
   {
      return pushDoorPanel;
   }
}
