package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;
import us.ihmc.rdx.imgui.ImGuiSliderDoubleWrapper;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;

import java.util.Set;

public class RDXArUcoMarkerNode extends ArUcoMarkerNode implements RDXSceneNodeInterface
{
   private final RDXDetectableSceneNodeBasics detectableSceneNodeBasics;
   private final ImGuiSliderDoubleWrapper alphaFilterValueSlider;

   public RDXArUcoMarkerNode(ArUcoMarkerNode nodeToCopy)
   {
      super(nodeToCopy.getMarkerID(),
            nodeToCopy.getName(),
            nodeToCopy.getMarkerID(),
            nodeToCopy.getMarkerSize());

      detectableSceneNodeBasics = new RDXDetectableSceneNodeBasics(this);

      alphaFilterValueSlider = new ImGuiSliderDoubleWrapper("Break frequency", "%.2f", 0.2, 5.0,
                                                            this::getBreakFrequency,
                                                            this::setBreakFrequency,
                                                            this::markModifiedByOperator);
   }

   @Override
   public void update()
   {
      detectableSceneNodeBasics.update();
   }

   @Override
   public void renderImGuiWidgets()
   {
      detectableSceneNodeBasics.renderImGuiWidgets();
      alphaFilterValueSlider.render();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      detectableSceneNodeBasics.getRenderables(renderables, pool, sceneLevels);
   }
}
