package us.ihmc.rdx.perception.scene;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.perception.scene.objects.RDXSceneObject;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;

import java.util.ArrayList;
import java.util.Set;

/**
 * Updates and renders perception scene objects.
 *
 * TODO: Create ROS2PerceptionSceneSubscription
 */
public class RDXPerceptionSceneUI
{
   private final ImGuiPanel panel = new ImGuiPanel("Objects Perception UI", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean showGraphics = new ImBoolean(true);
   private final ArrayList<RDXSceneObject> sceneObjects = new ArrayList<>();

   public void update()
   {
      for (RDXSceneObject sceneObject : sceneObjects)
      {
         sceneObject.update();
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.checkbox(labels.get("Show graphics"), showGraphics);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (showGraphics.get())
      {
         for (RDXSceneObject sceneObject : sceneObjects)
         {
            sceneObject.getRenderables(renderables, pool, sceneLevels);
         }
      }
   }

   public ImGuiPanel getPanel()
   {
      return panel;
   }

   public ArrayList<RDXSceneObject> getSceneObjects()
   {
      return sceneObjects;
   }
}