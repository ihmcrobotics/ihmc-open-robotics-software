package us.ihmc.rdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.VertexAttributes;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.shaders.DefaultShader;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import net.mgsx.gltf.scene3d.attributes.PBRColorAttribute;
import org.lwjgl.opengl.GL41;
import us.ihmc.rdx.shader.RDXShader;
import us.ihmc.rdx.shader.RDXUniform;

import javax.annotation.Nullable;

public abstract class AbstractRDXPointCloudRenderer implements RenderableProvider
{
   public enum ColoringMethod
   {
      DEFAULT, GRADIENT_WORLD_Z, GRADIENT_SENSOR_X, COLOR_IMAGE
   }

   // The point cloud renderable
   protected Renderable renderable;

   protected int floatsPerVertex = 0;

   // GENERALLY NEEDED UNIFORMS
   private float pointScale = 0.003f;
   private final Color defaultPointColor = new Color(Color.WHITE);
   private ColoringMethod coloringMethod = ColoringMethod.DEFAULT;

   /**
    * Creates and initializes everything needed for rendering point clouds.
    *
    * @param maxPoints Maximum number of points this renderer will be able to render.
    */
   public void create(int maxPoints)
   {
      GL41.glEnable(GL41.GL_VERTEX_PROGRAM_POINT_SIZE);

      renderable = new Renderable();
      renderable.meshPart.primitiveType = GL41.GL_POINTS;
      renderable.meshPart.offset = 0;
      renderable.material = new Material(PBRColorAttribute.createBaseColorFactor(Color.WHITE));

      VertexAttributes vertexAttributes = getVertexAttributes();
      floatsPerVertex = vertexAttributes.vertexSize / Float.BYTES;

      boolean isStatic = false;
      int maxIndices = 0;
      renderable.meshPart.mesh = new Mesh(isStatic, maxPoints, maxIndices, vertexAttributes);

      RDXShader shader = new RDXShader(getClass());
      shader.create(getVertexShaderFlags(), getFragmentShaderFlags());
      registerUniforms(shader);

      shader.init(renderable);
      renderable.shader = shader.getBaseShader();
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

   public void setPointScale(float scale)
   {
      pointScale = scale;
   }

   public void setDefaultPointColor(Color color)
   {
      defaultPointColor.set(color);
   }

   public void setColoringMethod(ColoringMethod method)
   {
      coloringMethod = method;
   }

   /**
    * @return The {@link ColoringMethod}s that the super class can use.
    */
   public abstract ColoringMethod[] getAvailableColoringMethods();

   /**
    * Called when initializing the shader.
    *
    * @return #define flags that should be added to the fragment shader.
    */
   @Nullable
   protected abstract String[] getFragmentShaderFlags();

   /**
    * Called when initializing the shader.
    *
    * @return #define flags that should be added to the vertex shader
    */
   @Nullable
   protected abstract String[] getVertexShaderFlags();

   /**
    * @return The vertex attributes of the shader
    */
   protected abstract VertexAttributes getVertexAttributes();

   /**
    * Registers all required uniforms to the passed in shader.
    *
    * @param rdxShader The shader to which uniforms are registered.
    */
   protected abstract void registerUniforms(RDXShader rdxShader);

   protected void registerGeneralUniforms(RDXShader rdxShader)
   {
      rdxShader.getBaseShader().register(DefaultShader.Inputs.viewTrans, DefaultShader.Setters.viewTrans);
      rdxShader.getBaseShader().register(DefaultShader.Inputs.projTrans, DefaultShader.Setters.projTrans);

      RDXUniform screenWidthUniform = RDXUniform.createGlobalUniform("u_screenWidth", (shader, inputID, renderable, combinedAttributes) ->
      {
         shader.set(inputID, shader.camera.viewportWidth);
      });
      rdxShader.registerUniform(screenWidthUniform);

      RDXUniform pointScaleUniform = RDXUniform.createGlobalUniform("u_pointScale", (shader, inputID, renderable, combinedAttributes) ->
      {
         shader.set(inputID, pointScale);
      });
      rdxShader.registerUniform(pointScaleUniform);

      RDXUniform defaultPointColorUniform = RDXUniform.createGlobalUniform("u_defaultPointColor", (shader, inputID, renderable, combinedAttributes) ->
      {
         shader.set(inputID, defaultPointColor);
      });
      rdxShader.registerUniform(defaultPointColorUniform);

      RDXUniform coloringMethodUniform = RDXUniform.createGlobalUniform("u_coloringMethod", (shader, inputID, renderable, combinedAttributes) ->
      {
         shader.set(inputID, coloringMethod.ordinal());
      });

      rdxShader.registerUniform(coloringMethodUniform);
   }
}
