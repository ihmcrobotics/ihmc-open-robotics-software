package us.ihmc.rdx.mesh;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;

/**
 * A performance optimization to help avoid rebuilding meshes unecessarily
 * and also to do this minimum amount required when necessary.
 * Best if using an extending class like {@link RDXMutableLineModel}.
 */
public class RDXMutableMeshModel
{
   private Color color = null;
   private RDXModelInstance modelInstance;

   public boolean isColorOutOfDate(Color color)
   {
      boolean outOfDate = this.color != color;
      this.color = color;
      return outOfDate;
   }

   protected void updateMesh(MeshDataHolder meshDataHolder)
   {
      boolean needToRebuildModel = modelInstance == null;

      Mesh mesh = null;
      if (!needToRebuildModel)
      {
         mesh = modelInstance.model.nodes.get(0).parts.get(0).meshPart.mesh;

         // These buffer capacities are final, so the Mesh needs to be recreated if there's not enough space
         needToRebuildModel |= mesh.getIndicesBuffer().capacity() < meshDataHolder.getTriangleIndices().length;
         needToRebuildModel |= mesh.getVerticesBuffer().capacity() < meshDataHolder.getVertices().length;
      }

      if (needToRebuildModel)
      {
         modelInstance = new RDXModelInstance(RDXModelBuilder.buildModelInstance(meshBuilder -> meshBuilder.addMesh(meshDataHolder, color)));
      }
      else
      {
         RDXMeshDataInterpreter.reorderMeshVertices(meshDataHolder, mesh);
         RDXMeshDataInterpreter.repositionMeshVertices(meshDataHolder, mesh, color);
      }
   }

   public RDXModelInstance getModelInstance()
   {
      return modelInstance;
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (modelInstance != null)
         modelInstance.getRenderables(renderables, pool);
   }
}