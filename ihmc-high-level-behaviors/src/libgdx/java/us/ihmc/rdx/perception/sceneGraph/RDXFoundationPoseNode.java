package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.foundationPose.FoundationPoseNode;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;

import java.util.Set;

public class RDXFoundationPoseNode extends RDXDetectableSceneNode
{
   private final FoundationPoseNode foundationPoseNode;
   private final RDXReferenceFrameGraphic poseGraphic;

   public RDXFoundationPoseNode(FoundationPoseNode foundationPoseNode)
   {
      super(foundationPoseNode);

      this.foundationPoseNode = foundationPoseNode;

      poseGraphic = new RDXReferenceFrameGraphic(0.2, Color.ORANGE);
      poseGraphic.setToReferenceFrame(foundationPoseNode.getNodeFrame());
   }

   @Override
   public void update(SceneGraph sceneGraph)
   {
      super.update(sceneGraph);
      poseGraphic.setToReferenceFrame(foundationPoseNode.getNodeFrame());
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (sceneLevels.contains(RDXSceneLevel.VIRTUAL) && !super.isGraphicsHidden())
      {
         poseGraphic.getRenderables(renderables, pool);
      }
   }

   @Override
   public void destroy()
   {
      super.destroy();
      poseGraphic.dispose();
   }
}
