package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.VertexAttribute;
import com.badlogic.gdx.graphics.VertexAttributes;
import com.badlogic.gdx.graphics.VertexAttributes.Usage;
import us.ihmc.rdx.AbstractRDXPointCloudRenderer;
import us.ihmc.rdx.shader.RDXShader;

import javax.annotation.Nullable;
import java.nio.FloatBuffer;

import static us.ihmc.rdx.AbstractRDXPointCloudRenderer.ColoringMethod.DEFAULT;

/**
 * Renders pre-extracted world-frame 3D points, each with a per-vertex RGB colour, via the
 * {@code RDXColoredWorldPointCloudRenderer.glsl} shader. Used to draw the fused voxel map with
 * colour encoding per-voxel occupancy probability.
 */
public class RDXColoredWorldPointCloudRenderer extends AbstractRDXPointCloudRenderer
{
   // Interleaved layout must match the shader: a_position (vec3, location 0) then a_color (vec3, location 1).
   private final VertexAttributes vertexAttributes = new VertexAttributes(VertexAttribute.Position(),
                                                                          new VertexAttribute(Usage.ColorUnpacked, 3, "a_color"));

   /**
    * Uploads {@code count} points with parallel RGB colours to the GPU mesh from flat arrays.
    *
    * @param positions World-frame coordinates, length &ge; {@code 3 * count} as [x0,y0,z0, x1,y1,z1, ...].
    * @param colors    RGB in [0, 1], length &ge; {@code 3 * count} as [r0,g0,b0, r1,g1,b1, ...].
    * @param count     Number of points to render (must not exceed the {@code maxPoints} of {@link #create(int)}).
    */
   public void updateMesh(float[] positions, float[] colors, int count)
   {
      FloatBuffer verticesBuffer = renderable.meshPart.mesh.getVerticesBuffer(true); // Mark dirty

      if (renderable.meshPart.size != count)
      {
         renderable.meshPart.size = count;
         verticesBuffer.limit(floatsPerVertex * count);
      }

      for (int i = 0; i < count; i++)
      {
         int offset = i * floatsPerVertex;
         int p = i * 3;
         verticesBuffer.put(offset,     positions[p]);
         verticesBuffer.put(offset + 1, positions[p + 1]);
         verticesBuffer.put(offset + 2, positions[p + 2]);
         verticesBuffer.put(offset + 3, colors[p]);
         verticesBuffer.put(offset + 4, colors[p + 1]);
         verticesBuffer.put(offset + 5, colors[p + 2]);
      }
   }

   @Override
   public ColoringMethod[] getAvailableColoringMethods()
   {
      return new ColoringMethod[] {DEFAULT};
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
