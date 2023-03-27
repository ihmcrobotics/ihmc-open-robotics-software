package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.perception.ObjectsPerceptionManager;
import us.ihmc.perception.objects.ObjectInfo;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;

import java.util.ArrayList;
import java.util.Set;

public class RDXObjectsPerceptionUI
{
   private final ImGuiPanel panel = new ImGuiPanel("Objects Perception UI", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean showGraphics = new ImBoolean(true);
   private final ROS2PublishSubscribeAPI ros2;
   private final ArrayList<RDXObjectPerceptionUpdater> objectUpdaters = new ArrayList<>();

   public RDXObjectsPerceptionUI(ROS2PublishSubscribeAPI ros2, ObjectInfo objectInfo)
   {
      this.ros2 = ros2;

      ArrayList<Integer> IDs = objectInfo.getIDs();
      for (int i = 0; i < IDs.size(); i++)
         objectUpdaters.add(new RDXObjectPerceptionUpdater(ros2, ObjectsPerceptionManager.DETECTED_OBJECT, IDs.get(i), objectInfo));
   }

   public void update()
   {
      for (RDXObjectPerceptionUpdater objectUpdater : objectUpdaters)
         objectUpdater.update();
   }

   public void renderImGuiWidgets()
   {
      ImGui.checkbox(labels.get("Show graphics"), showGraphics);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (showGraphics.get())
      {
         for (RDXObjectPerceptionUpdater objectUpdater : objectUpdaters)
            objectUpdater.getObject().getRenderables(renderables, pool, sceneLevels);
      }
   }

   public ImGuiPanel getPanel()
   {
      return panel;
   }
}