package us.ihmc.rdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.VertexAttribute;
import com.badlogic.gdx.graphics.VertexAttributes;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.shaders.DefaultShader;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import net.mgsx.gltf.scene3d.attributes.PBRColorAttribute;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.rdx.shader.RDXShader;
import us.ihmc.rdx.shader.RDXUniform;

import java.nio.FloatBuffer;
import java.util.List;
import java.util.stream.IntStream;

public class RDXPointCloudRenderer implements RenderableProvider
{
   public enum ColoringMethod
   {
      DEFAULT, GRADIENT_WORLD_Z
   }

   private Renderable renderable;

   private final VertexAttributes vertexAttributes = new VertexAttributes(VertexAttribute.Position());
   private final int floatsPerVertex = vertexAttributes.vertexSize / Float.BYTES;

   // GENERALLY NEEDED UNIFORMS
   private final RDXUniform screenWidthUniform = RDXUniform.createGlobalUniform("u_screenWidth", (shader, inputID, renderable, combinedAttributes) ->
   {
      shader.set(inputID, shader.camera.viewportWidth);
   });

   private float pointScale = 0.01f;
   private final RDXUniform pointScaleUniform = RDXUniform.createGlobalUniform("u_pointScale", (shader, inputID, renderable, combinedAttributes) ->
   {
      shader.set(inputID, pointScale);
   });

   private final Color defaultPointColor = new Color(Color.WHITE);
   private final RDXUniform defaultPointColorUniform = RDXUniform.createGlobalUniform("u_defaultPointColor", (shader, inputID, renderable, combinedAttributes) ->
   {
      shader.set(inputID, defaultPointColor);
   });

   private ColoringMethod coloringMethod = ColoringMethod.DEFAULT;
   private final RDXUniform coloringMethodUniform = RDXUniform.createGlobalUniform("u_coloringMethod", (shader, inputID, renderable, combinedAttributes) ->
   {
      shader.set(inputID, coloringMethod.ordinal());
   });

   public void create(int maxPoints)
   {
      GL41.glEnable(GL41.GL_VERTEX_PROGRAM_POINT_SIZE);

      renderable = new Renderable();
      renderable.meshPart.primitiveType = GL41.GL_POINTS;
      renderable.meshPart.offset = 0;
      renderable.material = new Material(PBRColorAttribute.createBaseColorFactor(Color.WHITE));

      boolean isStatic = false;
      int maxIndices = 0;
      renderable.meshPart.mesh = new Mesh(isStatic, maxPoints, maxIndices, vertexAttributes);

      RDXShader shader = new RDXShader(getClass());
      shader.create();
      shader.getBaseShader().register(DefaultShader.Inputs.viewTrans, DefaultShader.Setters.viewTrans);
      shader.getBaseShader().register(DefaultShader.Inputs.projTrans, DefaultShader.Setters.projTrans);
      shader.registerUniform(screenWidthUniform);
      shader.registerUniform(pointScaleUniform);
      shader.registerUniform(defaultPointColorUniform);
      shader.registerUniform(coloringMethodUniform);

      shader.init(renderable);
      renderable.shader = shader.getBaseShader();
   }

   public void updateMesh(List<? extends Point3DReadOnly> points)
   {
      // Ensure correct length and initialize vertices buffer
      if (renderable.meshPart.mesh.getVerticesBuffer(false).limit() != (floatsPerVertex * points.size()))
      {
         renderable.meshPart.mesh.setVertices(new float[floatsPerVertex * points.size()]);
         renderable.meshPart.size = points.size();
      }

      // Copy points over
      FloatBuffer verticesBuffer = renderable.meshPart.mesh.getVerticesBuffer(true); // Mark dirty
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
      // Ensure correct length and initialize vertices buffer
      if (renderable.meshPart.mesh.getVerticesBuffer(false).limit() != (floatsPerVertex * points.length))
      {
         renderable.meshPart.mesh.setVertices(new float[floatsPerVertex * points.length]);
         renderable.meshPart.size = points.length;
      }

      // Copy points over
      FloatBuffer verticesBuffer = renderable.meshPart.mesh.getVerticesBuffer(true); // Mark dirty
      IntStream.range(0, points.length).parallel().unordered().forEach(i ->
      {
         int offset = i * floatsPerVertex;
         verticesBuffer.put(offset, points[i].getX32());
         verticesBuffer.put(offset + 1, points[i].getY32());
         verticesBuffer.put(offset + 2, points[i].getZ32());
      });
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (renderable != null)
         renderables.add(renderable);
   }

   public void dispose()
   {
      if (renderable.meshPart.mesh != null)
         renderable.meshPart.mesh.dispose();
   }

   public void setPointScale(float size)
   {
      pointScale = size;
   }

   public void setDefaultPointColor(Color color)
   {
      defaultPointColor.set(color);
   }

   public void setColoringMethod(ColoringMethod method)
   {
      coloringMethod = method;
   }
}