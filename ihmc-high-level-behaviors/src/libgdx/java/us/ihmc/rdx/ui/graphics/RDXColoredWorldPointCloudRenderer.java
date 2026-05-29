package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.VertexAttribute;
import com.badlogic.gdx.graphics.VertexAttributes;
import com.badlogic.gdx.graphics.VertexAttributes.Usage;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import net.mgsx.gltf.scene3d.attributes.PBRColorAttribute;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import org.lwjgl.opengl.GL41;
import com.badlogic.gdx.graphics.g3d.shaders.DefaultShader;
import us.ihmc.rdx.shader.RDXShader;
import us.ihmc.rdx.shader.RDXUniform;

import java.nio.FloatBuffer;
import java.util.List;
import java.util.stream.IntStream;

/**
 * Renders pre-extracted world-frame 3D points coloured with per-vertex RGB sampled from the
 * source camera image. Vertex layout: 6 floats per point — (x, y, z, r, g, b).
 *
 * <p>Shader: {@code RDXColoredWorldPointCloudRenderer.glsl}
 */
public class RDXColoredWorldPointCloudRenderer implements RenderableProvider
{
   private static final int FLOATS_PER_VERTEX = 6; // (x,y,z) + (r,g,b)

   private Renderable renderable;
   private float pointScale = 0.003f;

   /** Call once on the render thread before first use. */
   public void create(int maxPoints)
   {
      GL41.glEnable(GL41.GL_VERTEX_PROGRAM_POINT_SIZE);

      renderable = new Renderable();
      renderable.meshPart.primitiveType = GL41.GL_POINTS;
      renderable.meshPart.offset = 0;
      renderable.material = new Material(PBRColorAttribute.createBaseColorFactor(Color.WHITE));

      VertexAttributes attributes = new VertexAttributes(
            VertexAttribute.Position(),                           // a_position (location 0)
            new VertexAttribute(Usage.Generic, 3, "a_color"));   // a_color    (location 1)

      renderable.meshPart.mesh = new Mesh(false, maxPoints, 0, attributes);

      RDXShader shader = new RDXShader(getClass());
      shader.create(null, null);
      registerUniforms(shader);
      shader.init(renderable);
      renderable.shader = shader.getBaseShader();
   }

   /**
    * Updates the vertex buffer with a new set of coloured world-frame points.
    *
    * @param points Pre-extracted world-frame positions.
    * @param colors Parallel RGB values in [0,1]. Must have the same size as {@code points}.
    */
   public void updateMesh(List<float[]> points, List<float[]> colors)
   {
      if (renderable == null)
         return;

      int count = Math.min(points.size(), colors.size());
      FloatBuffer buf = renderable.meshPart.mesh.getVerticesBuffer(true);

      if (renderable.meshPart.size != count)
      {
         renderable.meshPart.size = count;
         buf.limit(FLOATS_PER_VERTEX * count);
      }

      IntStream.range(0, count).parallel().unordered().forEach(i ->
      {
         float[] p = points.get(i);
         float[] c = colors.get(i);
         int off = i * FLOATS_PER_VERTEX;
         buf.put(off,     p[0]);
         buf.put(off + 1, p[1]);
         buf.put(off + 2, p[2]);
         buf.put(off + 3, c[0]);
         buf.put(off + 4, c[1]);
         buf.put(off + 5, c[2]);
      });
   }

   public void setPointScale(float scale)
   {
      pointScale = scale;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (renderable != null)
         renderables.add(renderable);
   }

   public void dispose()
   {
      if (renderable != null && renderable.meshPart.mesh != null)
         renderable.meshPart.mesh.dispose();
   }

   private void registerUniforms(RDXShader rdxShader)
   {
      rdxShader.getBaseShader().register(
            DefaultShader.Inputs.viewTrans,
            DefaultShader.Setters.viewTrans);
      rdxShader.getBaseShader().register(
            DefaultShader.Inputs.projTrans,
            DefaultShader.Setters.projTrans);

      rdxShader.registerUniform(RDXUniform.createGlobalUniform("u_screenWidth",
            (shader, id, r, attrs) -> shader.set(id, shader.camera.viewportWidth)));
      rdxShader.registerUniform(RDXUniform.createGlobalUniform("u_pointScale",
            (shader, id, r, attrs) -> shader.set(id, pointScale)));
   }
}
