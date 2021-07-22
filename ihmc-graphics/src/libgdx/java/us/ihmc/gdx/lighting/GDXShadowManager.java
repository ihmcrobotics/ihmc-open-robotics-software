package us.ihmc.gdx.lighting;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.Shader;
import com.badlogic.gdx.graphics.g3d.utils.DefaultShaderProvider;
import com.badlogic.gdx.graphics.glutils.FrameBuffer;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import com.badlogic.gdx.utils.Array;

public class GDXShadowManager
{
   protected final Array<GDXLight> lights = new Array<>();
   private final ShaderProgram shader;
   private final ModelBatch batch;

   private FrameBuffer framebuffer;

   public GDXShadowManager()
   {
      this(GDXShadowMapShader.buildShaderProgram());
   }

   /**
    * @param shader The ShaderProgram used to create the shader used by the main ModelBatch
    */
   public GDXShadowManager(ShaderProgram shader)
   {
      final GDXShadowManager manager = this;

      this.shader = shader;
      this.batch = new ModelBatch(new DefaultShaderProvider()
      {
         @Override
         protected Shader createShader(final Renderable renderable)
         {
            return new GDXShadowMapShader(renderable, shader, manager);
         }
      });
   }

   public static String getVertexShader()
   {
      return Gdx.files.classpath("us/ihmc/gdx/shadows/scene_v.glsl").readString();
   }

   public static String getFragmentShader()
   {
      return Gdx.files.classpath("us/ihmc/gdx/shadows/scene_f.glsl").readString();
   }

   public void addLight(GDXLight light)
   {
      lights.add(light);
   }

   /**
    * Ensures that all lights are initialized
    */
   public void update()
   {
      for (GDXLight light : lights)
      {
         if (!light.isInitialized())
            light.init();
      }
   }

   /**
    * Renders shadows to the ModelBatch and prepares the OpenGL environment for the rendering of the main ModelBatch, which should be rendered immediately after
    * this call.
    *
    * @param renderableProviders The models to be rendered
    * @param program             The {@link GDXSceneShader} belonging to the main batch
    */
   public <T extends RenderableProvider> void renderShadows(Camera camera, Iterable<T> renderableProviders, ShaderProgram program)
   {
      for (GDXLight light : lights)
      {
         light.render(renderableProviders);
      }

      if (framebuffer == null)
      {
         framebuffer = new FrameBuffer(Pixmap.Format.RGBA8888, Gdx.graphics.getWidth(), Gdx.graphics.getHeight(), true);
      }

      framebuffer.begin();

      Gdx.gl.glClearColor(0.4f, 0.4f, 0.4f, 0.4f);
      Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT | GL20.GL_DEPTH_BUFFER_BIT);

      batch.begin(camera);
      batch.render(renderableProviders);
      batch.end();

      framebuffer.end();

      program.begin();
      Texture shadows = framebuffer.getColorBufferTexture();
      final int textureNum = shadows.getTextureObjectHandle();
      shadows.bind(textureNum);
      program.setUniformi("u_shadows", textureNum);
      program.setUniformf("u_screenWidth", Gdx.graphics.getWidth());
      program.setUniformf("u_screenHeight", Gdx.graphics.getHeight());
      program.end();
   }
}
