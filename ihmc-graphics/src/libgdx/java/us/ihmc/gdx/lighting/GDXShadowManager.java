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
import com.badlogic.gdx.utils.Pool;
import us.ihmc.log.LogTools;

import java.util.HashSet;
import java.util.function.Supplier;

public class GDXShadowManager
{
   private final ShaderProgram shader;
   private final ModelBatch batch;
   private final Array<Renderable> renderableArray = new Array<>();
   private final Pool<Renderable> renderablePool = new Pool<Renderable>()
   {
      @Override
      protected Renderable newObject()
      {
         return new Renderable();
      }
   };
   private final float antiAliasing;
   private final Supplier<Float> ambientLightSupplier;
   private boolean useViewport = false;
   private int x = 0;
   private int y = 0;
   private int width = 0;
   private int height = 0;
   private FrameBuffer framebuffer;
   protected final HashSet<GDXLight> additionalOffscreenLights = new HashSet<>();

   public GDXShadowManager(Supplier<Iterable<GDXLight>> lightSupplier, Supplier<Float> ambientLightSupplier)
   {
      this(1.0f, lightSupplier, ambientLightSupplier);
   }

   public GDXShadowManager(float antiAliasing, Supplier<Iterable<GDXLight>> lightSupplier, Supplier<Float> ambientLightSupplier)
   {
      this(antiAliasing, GDXShadowMapShader.buildShaderProgram(), lightSupplier, ambientLightSupplier);
   }

   /**
    * @param shader The ShaderProgram used to create the shader used by the main ModelBatch
    */
   public GDXShadowManager(float antiAliasing, ShaderProgram shader,
                           Supplier<Iterable<GDXLight>> lightSupplier,
                           Supplier<Float> ambientLightSupplier)
   {
      this.ambientLightSupplier = ambientLightSupplier;
      final GDXShadowManager manager = this;

      this.antiAliasing = antiAliasing;

      this.shader = shader;
      this.batch = new ModelBatch(new DefaultShaderProvider()
      {
         @Override
         protected Shader createShader(final Renderable renderable)
         {
            return new GDXShadowMapShader(renderable, shader, manager, lightSupplier);
         }
      });

      //Add three lights offscreen so that things render properly. I do not know why this is necessary, and am not proud of it.
      additionalOffscreenLights.add(new GDXPointLight(0.0, 0.0, -500.0));
      additionalOffscreenLights.add(new GDXPointLight(1.0, 0.0, -500.0));
      additionalOffscreenLights.add(new GDXPointLight(2.0, 0.0, -500.0));
   }

   public static String getVertexShader()
   {
      return Gdx.files.classpath("us/ihmc/gdx/shadows/scene_v.glsl").readString();
   }

   public static String getFragmentShader()
   {
      return Gdx.files.classpath("us/ihmc/gdx/shadows/scene_f.glsl").readString();
   }

   /**
    * Renders shadows to the ModelBatch and prepares the OpenGL environment for the rendering of the main ModelBatch, which should be rendered immediately after
    * this call.
    *
    * @param renderableProviders The models to be rendered
    */
   public <T extends RenderableProvider> void renderShadows(Iterable<GDXLight> lights, Camera camera, Iterable<T> renderableProviders)
   {
      renderShadows(lights, camera, renderableProviders, Gdx.graphics.getWidth(), Gdx.graphics.getHeight());
   }

   /**
    * Renders shadows to the ModelBatch and prepares the OpenGL environment for the rendering of the main ModelBatch, which should be rendered immediately after
    * this call.
    *
    * @param renderableProviders The models to be rendered
    */
   public <T extends RenderableProvider> void renderShadows(Iterable<GDXLight> lights, Camera camera, Iterable<T> renderableProviders, int width, int height)
   {
      for (GDXLight additionalOffscreenLight : additionalOffscreenLights)
      {
         additionalOffscreenLight.render(renderableProviders);
      }
      for (GDXLight light : lights)
      {
         light.render(renderableProviders);
      }

      if ((framebuffer == null || width != framebuffer.getWidth() || height != framebuffer.getHeight()) && width > 0 && height > 0)
      {
         if (framebuffer != null)
            framebuffer.dispose();

         LogTools.info("Allocating framebuffer of size: " + width + "x" + height);
         framebuffer = new FrameBuffer(Pixmap.Format.RGBA8888, width, height, true);
      }

      framebuffer.begin();

      if (useViewport)
         Gdx.gl.glViewport(this.x, this.y, this.width, this.height);

      Gdx.gl.glClearColor(0.4f, 0.4f, 0.4f, 0.4f);
      Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT | GL20.GL_DEPTH_BUFFER_BIT);

      renderablePool.freeAll(renderableArray);
      renderableArray.clear();

      for (RenderableProvider renderableProvider : renderableProviders)
      {
         renderableProvider.getRenderables(renderableArray, renderablePool);
      }

      //We must individually render every renderable here or everything breaks and things are bad. It's annoying, but there is no performance hit, so it's okay.
      for (Renderable renderable : renderableArray)
      {
         batch.begin(camera);
         batch.render(renderable);
         batch.end();
      }

      framebuffer.end();
   }

   public void apply(ShaderProgram program)
   {
      program.begin();
      Texture shadows = framebuffer.getColorBufferTexture();
      int textureNum = shadows.getTextureObjectHandle();
      shadows.bind(textureNum);
      program.setUniformi("u_shadows", textureNum);
      program.setUniformf("u_screenWidth", Gdx.graphics.getWidth());
      program.setUniformf("u_screenHeight", Gdx.graphics.getHeight());
      program.setUniformf("u_antiAliasing", antiAliasing);
      program.setUniformf("u_ambientLight", ambientLightSupplier.get());
      program.end();
   }

   public void setViewportBounds(int x, int y, int width, int height)
   {
      useViewport = true;
      this.x = x;
      this.y = y;
      this.width = width;
      this.height = height;
   }
}
