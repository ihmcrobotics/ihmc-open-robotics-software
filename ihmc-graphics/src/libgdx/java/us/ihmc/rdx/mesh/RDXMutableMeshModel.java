package us.ihmc.rdx.mesh;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.graphicsDescription.MeshDataBuilder;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;

import java.util.function.Consumer;

/**
 * A performance optimization to help avoid rebuilding meshes unecessarily
 * and also to do this minimum amount required when necessary.
 * Best if using an extending class like {@link RDXMutableLineModel}.
 */
public class RDXMutableMeshModel
{
   private Color color = null;
   protected RDXModelInstance modelInstance;

   private transient final MeshDataHolder emptyMeshDataHolder = new MeshDataBuilder().generateMeshDataHolder();

   public boolean isColorOutOfDate(Color color)
   {
      boolean outOfDate = this.color != color;
      this.color = color;
      return outOfDate;
   }

   protected void updateMesh(MeshDataHolder meshDataHolder)
   {
      boolean emptyModel = meshDataHolder.getTriangleIndices().length == 0;
      emptyModel |= meshDataHolder.getVertices().length == 0;

      if (emptyModel)
      {
         modelInstance = null;
      }
      else
      {
         boolean initialOrLargerModelNeeded = modelInstance == null;

         Mesh mesh = null;
         if (!initialOrLargerModelNeeded)
         {
            mesh = modelInstance.model.nodes.get(0).parts.get(0).meshPart.mesh;

            // These buffer capacities are final, so the Mesh needs to be recreated if there's not enough space
            initialOrLargerModelNeeded |= mesh.getIndicesBuffer().capacity() < meshDataHolder.getTriangleIndices().length;
            initialOrLargerModelNeeded |= mesh.getVerticesBuffer().capacity() < meshDataHolder.getVertices().length;
         }

         if (initialOrLargerModelNeeded)
         {
            modelInstance = new RDXModelInstance(RDXModelBuilder.buildModelInstance(meshBuilder -> meshBuilder.addMesh(meshDataHolder, color)));
         }
         else
         {
            RDXMeshDataInterpreter.reorderMeshVertices(meshDataHolder, mesh);
            RDXMeshDataInterpreter.repositionMeshVertices(meshDataHolder, mesh, color);
         }
      }
   }

   public void clear()
   {
      updateMesh(emptyMeshDataHolder);
   }

   public void accessModelIfExists(Consumer<RDXModelInstance> modelInstanceAccessor)
   {
      if (modelInstance != null)
      {
         modelInstanceAccessor.accept(modelInstance);
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (modelInstance != null)
         modelInstance.getRenderables(renderables, pool);
   }
}