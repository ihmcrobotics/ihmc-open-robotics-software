package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.rdx.imgui.ImGuiEnumPlot;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;

import java.util.Set;

public class RDXDetectableSceneNodeBasics
{
   private final DetectableSceneNode detectableSceneNode;
   private final RDXSceneNodeBasics sceneNodeBasics;
   private final ImGuiEnumPlot currentlyDetectedPlot;

   public RDXDetectableSceneNodeBasics(DetectableSceneNode detectableSceneNode)
   {
      this.detectableSceneNode = detectableSceneNode;
      sceneNodeBasics = new RDXSceneNodeBasics(detectableSceneNode);
      currentlyDetectedPlot = new ImGuiEnumPlot();
   }

   public void update()
   {
      sceneNodeBasics.update();
   }

   public void renderImGuiWidgets()
   {
      sceneNodeBasics.renderImGuiWidgets();
      ImGui.sameLine();

      boolean currentlyDetected = detectableSceneNode.getCurrentlyDetected();
      currentlyDetectedPlot.setWidgetTextColor(currentlyDetected ? ImGuiTools.GREEN : ImGuiTools.RED);
      currentlyDetectedPlot.render(currentlyDetected ? 1 : 0, currentlyDetected ? "CURRENTLY DETECTED" : "NOT DETECTED");
   }

   public void renderRemove(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      sceneNodeBasics.renderRemove(modificationQueue, sceneGraph);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      sceneNodeBasics.getRenderables(renderables, pool, sceneLevels);
   }
}
