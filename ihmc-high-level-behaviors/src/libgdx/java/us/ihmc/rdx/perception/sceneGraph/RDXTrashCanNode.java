package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.type.ImDouble;
import us.ihmc.commons.MathTools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.rigidBody.trashcan.TrashCanNode;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;

import java.util.Set;

public class RDXTrashCanNode extends RDXDetectableSceneNode
{
   private final TrashCanNode trashCanNode;

   private final RDXReferenceFrameGraphic trashCanFrameGraphic = new RDXReferenceFrameGraphic(0.2, Color.BLUE);
   private final ImDouble trashCanYawDegrees = new ImDouble(0.0f);

   public RDXTrashCanNode(TrashCanNode trashCanNode)
   {
      super(trashCanNode);
      this.trashCanNode = trashCanNode;

      trashCanFrameGraphic.setToReferenceFrame(trashCanNode.getTrashCanFrame());
   }

   @Override
   public void update(SceneGraph sceneGraph)
   {
      trashCanFrameGraphic.updateFromLastGivenFrame();
   }

   @Override
   public void renderImGuiWidgets(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      super.renderImGuiWidgets(modificationQueue, sceneGraph);

      if (ImGuiTools.volatileInputDouble("Yaw (degrees)", trashCanYawDegrees))
      {
         trashCanYawDegrees.set(MathTools.clamp(trashCanYawDegrees.get(), 0.0, 360.0));
         trashCanNode.setYaw(Math.toRadians(trashCanYawDegrees.get()));
         trashCanNode.freeze();
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      super.getRenderables(renderables, pool, sceneLevels);

      if (!trashCanFrameGraphic.getFramePose3D().containsNaN())
         trashCanFrameGraphic.getRenderables(renderables, pool);
      else
         LogTools.warn("TrashCan Node pose not valid");
   }
}
