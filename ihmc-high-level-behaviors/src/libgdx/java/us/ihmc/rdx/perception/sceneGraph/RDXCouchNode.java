package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.rigidBody.couch.CouchNode;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;

import java.util.Set;

public class RDXCouchNode extends RDXDetectableSceneNode
{
   private final RDXReferenceFrameGraphic couchCentroidFrameGraphic = new RDXReferenceFrameGraphic(0.2, Color.BLUE);
   private final RDXReferenceFrameGraphic pillowCentroidFrameGraphic = new RDXReferenceFrameGraphic(0.2, Color.RED);
   private final RDXReferenceFrameGraphic couchCornerFrameGraphic = new RDXReferenceFrameGraphic(0.2, Color.BLACK);

   public RDXCouchNode(CouchNode couchNode)
   {
      super(couchNode);

      couchCentroidFrameGraphic.setToReferenceFrame(couchNode.getCouchCentroidFrame());
      pillowCentroidFrameGraphic.setToReferenceFrame(couchNode.getPillowFrame());
      couchCornerFrameGraphic.setToReferenceFrame(couchNode.getCornerFrame());
   }

   @Override
   public void update(SceneGraph sceneGraph)
   {
      super.update(sceneGraph);
      couchCentroidFrameGraphic.updateFromLastGivenFrame();
      pillowCentroidFrameGraphic.updateFromLastGivenFrame();
      couchCornerFrameGraphic.updateFromLastGivenFrame();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      super.getRenderables(renderables, pool, sceneLevels);

      if (!couchCentroidFrameGraphic.getFramePose3D().containsNaN())
         couchCentroidFrameGraphic.getRenderables(renderables, pool);
      if (!pillowCentroidFrameGraphic.getFramePose3D().containsNaN())
         pillowCentroidFrameGraphic.getRenderables(renderables, pool);
      if (!couchCornerFrameGraphic.getFramePose3D().containsNaN())
         couchCornerFrameGraphic.getRenderables(renderables, pool);
   }
}
