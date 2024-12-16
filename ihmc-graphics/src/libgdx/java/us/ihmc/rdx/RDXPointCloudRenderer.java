package us.ihmc.rdx;

import com.badlogic.gdx.graphics.VertexAttribute;
import com.badlogic.gdx.graphics.VertexAttributes;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.rdx.shader.RDXShader;

import javax.annotation.Nullable;
import java.nio.FloatBuffer;
import java.util.List;
import java.util.stream.IntStream;

import static us.ihmc.rdx.AbstractRDXPointCloudRenderer.ColoringMethod.DEFAULT;
import static us.ihmc.rdx.AbstractRDXPointCloudRenderer.ColoringMethod.GRADIENT_WORLD_Z;

public class RDXPointCloudRenderer extends AbstractRDXPointCloudRenderer
{
   private final VertexAttributes vertexAttributes = new VertexAttributes(VertexAttribute.Position());

   public void updateMesh(List<? extends Point3DReadOnly> points)
   {
      FloatBuffer verticesBuffer = renderable.meshPart.mesh.getVerticesBuffer(true); // Mark dirty

      // Ensure correct length
      int pointCloudSize = points.size();
      if (renderable.meshPart.size != pointCloudSize)
      {
         renderable.meshPart.size = pointCloudSize;
         verticesBuffer.limit(floatsPerVertex * pointCloudSize);
      }

      // Copy points over
      IntStream.range(0, points.size()).parallel().unordered().forEach(i ->
      {
         int offset = i * floatsPerVertex;
         verticesBuffer.put(offset, points.get(i).getX32());
         verticesBuffer.put(offset + 1, points.get(i).getY32());
         verticesBuffer.put(offset + 2, points.get(i).getZ32());
      });
   }

   public void updateMesh(Point3DReadOnly[] points)
   {
      FloatBuffer verticesBuffer = renderable.meshPart.mesh.getVerticesBuffer(true); // Mark dirty

      // Ensure correct length
      int pointCloudSize = points.length;
      if (renderable.meshPart.size != pointCloudSize)
      {
         renderable.meshPart.size = pointCloudSize;
         verticesBuffer.limit(floatsPerVertex * pointCloudSize);
      }

      // Copy points over
      IntStream.range(0, points.length).parallel().unordered().forEach(i ->
      {
         int offset = i * floatsPerVertex;
         verticesBuffer.put(offset, points[i].getX32());
         verticesBuffer.put(offset + 1, points[i].getY32());
         verticesBuffer.put(offset + 2, points[i].getZ32());
      });
   }

   @Override
   public ColoringMethod[] getAvailableColoringMethods()
   {
      return new ColoringMethod[] {DEFAULT, GRADIENT_WORLD_Z};
   }

   @Nullable
   @Override
   protected String[] getFragmentShaderFlags()
   {
      return null;
   }

   @Nullable
   @Override
   protected String[] getVertexShaderFlags()
   {
      return null;
   }

   @Override
   protected VertexAttributes getVertexAttributes()
   {
      return vertexAttributes;
   }

   @Override
   protected void registerUniforms(RDXShader rdxShader)
   {
      registerGeneralUniforms(rdxShader);
   }
}