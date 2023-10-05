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

public class RDXDetectableSceneNode extends RDXSceneNode
{
   private final DetectableSceneNode detectableSceneNode;
   private final RDXSceneNode uiSceneNode;
   private final ImGuiEnumPlot currentlyDetectedPlot;

   public RDXDetectableSceneNode(DetectableSceneNode detectableSceneNode)
   {
      super(detectableSceneNode);

      this.detectableSceneNode = detectableSceneNode;
      uiSceneNode = new RDXSceneNode(detectableSceneNode);
      currentlyDetectedPlot = new ImGuiEnumPlot();
   }

   @Override
   public void update(SceneGraphModificationQueue modificationQueue)
   {
      uiSceneNode.update(modificationQueue);
   }

   @Override
   public void renderImGuiWidgets()
   {
      uiSceneNode.renderImGuiWidgets();
      ImGui.sameLine();

      boolean currentlyDetected = detectableSceneNode.getCurrentlyDetected();
      currentlyDetectedPlot.setWidgetTextColor(currentlyDetected ? ImGuiTools.GREEN : ImGuiTools.RED);
      currentlyDetectedPlot.render(currentlyDetected ? 1 : 0, currentlyDetected ? "CURRENTLY DETECTED" : "NOT DETECTED");
   }

   @Override
   public void renderRemove(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      uiSceneNode.renderRemove(modificationQueue, sceneGraph);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      uiSceneNode.getRenderables(renderables, pool, sceneLevels);
   }
}
