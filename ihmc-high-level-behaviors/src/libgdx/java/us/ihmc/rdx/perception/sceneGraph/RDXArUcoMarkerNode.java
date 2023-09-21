package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.perception.sceneGraph.SceneGraphNodeMove;
import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;
import us.ihmc.rdx.imgui.ImGuiInputDoubleWrapper;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;

import java.util.List;
import java.util.Set;

public class RDXArUcoMarkerNode extends ArUcoMarkerNode implements RDXSceneNodeInterface
{
   private final RDXDetectableSceneNodeBasics detectableSceneNodeBasics;
   private final ImGuiInputDoubleWrapper alphaFilterValueSlider;

   public RDXArUcoMarkerNode(ArUcoMarkerNode nodeToCopy)
   {
      super(nodeToCopy.getID(),
            nodeToCopy.getName(),
            nodeToCopy.getMarkerID(),
            nodeToCopy.getMarkerSize());

      detectableSceneNodeBasics = new RDXDetectableSceneNodeBasics(this);

      alphaFilterValueSlider = new ImGuiInputDoubleWrapper("Break frequency:", "%.2f", 0.2, 5.0,
                                                           this::getBreakFrequency,
                                                           this::setBreakFrequency,
                                                           this::freezeFromModification);
      alphaFilterValueSlider.setWidgetWidth(100.0f);
   }

   @Override
   public void update(List<SceneGraphNodeMove> sceneGraphNodeMoves)
   {
      detectableSceneNodeBasics.update();
   }

   @Override
   public void renderImGuiWidgets()
   {
      detectableSceneNodeBasics.renderImGuiWidgets();
      ImGui.text("Marker ID: %d   Size: %.2f m".formatted(getMarkerID(), getMarkerSize()));
      ImGui.sameLine();
      alphaFilterValueSlider.render();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      detectableSceneNodeBasics.getRenderables(renderables, pool, sceneLevels);
   }
}
