package us.ihmc.rdx.lighting;

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
import org.lwjgl.opengl.GL41;
import us.ihmc.log.LogTools;

import java.util.ArrayList;

public class GDXShadowManager
{
   private final float antiAliasing;
   private boolean useViewport = false;
   private int x = 0;
   private int y = 0;
   private int width = 0;
   private int height = 0;
   private FrameBuffer framebuffer;
   private final ArrayList<GDXPointLight> additionalOffscreenLights = new ArrayList<>();
   private final ArrayList<GDXPointLight> pointLights = new ArrayList<>();
   private final ArrayList<GDXDirectionalLight> directionalLights = new ArrayList<>();
   private final ShaderProgram shadowSceneShaderProgram;
   private final ModelBatch shadowMapBatch;
   private final ModelBatch shadowSceneBatch;
   private float ambientLight;

   public GDXShadowManager(float antiAliasing, float ambientLight)
   {
      this.antiAliasing = antiAliasing;
      this.ambientLight = ambientLight;

      String vertexShader = Gdx.files.classpath("us/ihmc/rdx/shadows/scene_v.glsl").readString();
      String fragmentShader = Gdx.files.classpath("us/ihmc/rdx/shadows/scene_f.glsl").readString();
      shadowSceneShaderProgram = new ShaderProgram(vertexShader, fragmentShader);
      shadowSceneBatch = new ModelBatch(new DefaultShaderProvider()
      {
         @Override
         protected Shader createShader(Renderable renderable)
         {
            return new GDXShadowSceneShader(renderable, shadowSceneShaderProgram);
         }
      });

      shadowMapBatch = new ModelBatch(new DefaultShaderProvider()
      {
         @Override
         protected Shader createShader(final Renderable renderable)
         {
            return new GDXShadowMapShader(renderable, GDXShadowManager.this);
         }
      });

      //Add three lights offscreen so that things render properly. I do not know why this is necessary, and am not proud of it.
      GDXPointLight lightOne = new GDXPointLight();
      lightOne.getPosition().set(0.0, 0.0, -500.0);
      additionalOffscreenLights.add(lightOne);
      GDXPointLight lightTwo = new GDXPointLight();
      lightTwo.getPosition().set(1.0, 0.0, -500.0);
      additionalOffscreenLights.add(lightTwo);
      GDXPointLight lightThree = new GDXPointLight();
      lightThree.getPosition().set(2.0, 0.0, -500.0);
      additionalOffscreenLights.add(lightThree);
   }

   /**
    * Renders shadows to the ModelBatch and prepares the OpenGL environment for the rendering of the main ModelBatch,
    * which should be rendered immediately after this call.
    *
    * @param renderableProviders The models to be rendered
    */
   public <T extends RenderableProvider> void renderShadows(Camera camera, Iterable<T> renderableProviders)
   {
      renderShadows(camera, renderableProviders, Gdx.graphics.getWidth(), Gdx.graphics.getHeight());
   }

   /**
    * Renders shadows to the ModelBatch and prepares the OpenGL environment for the rendering of the main ModelBatch,
    * which should be rendered immediately after this call.
    *
    * @param renderableProviders The models to be rendered
    */
   public <T extends RenderableProvider> void renderShadows(Camera camera, Iterable<T> renderableProviders, int width, int height)
   {
      for (GDXPointLight additionalOffscreenLight : additionalOffscreenLights)
      {
         additionalOffscreenLight.render(renderableProviders);
      }
      for (GDXPointLight pointLight : pointLights)
      {
         pointLight.render(renderableProviders);
      }
      for (GDXDirectionalLight directionalLight : directionalLights)
      {
         directionalLight.render(renderableProviders);
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
         GL41.glViewport(this.x, this.y, this.width, this.height);

      GL41.glClearColor(0.4f, 0.4f, 0.4f, 0.4f);
      GL41.glClear(GL20.GL_COLOR_BUFFER_BIT | GL20.GL_DEPTH_BUFFER_BIT);

      // We must individually render every renderable here or everything breaks and things are bad.
      // It's annoying, but there is no performance hit, so it's okay.
      for (RenderableProvider renderable : renderableProviders)
      {
         shadowMapBatch.begin(camera);
         shadowMapBatch.render(renderable);
         shadowMapBatch.end();
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
      program.setUniformf("u_ambientLight", ambientLight);
      program.end();
   }

   public void preRender(Camera camera)
   {
      apply(shadowSceneShaderProgram);
      shadowSceneBatch.begin(camera);
   }

   public void render(Iterable<? extends RenderableProvider> renderables)
   {
      shadowSceneBatch.render(renderables);
   }

   public void postRender()
   {
      shadowSceneBatch.end();
   }

   public void setViewportBounds(int x, int y, int width, int height)
   {
      useViewport = true;
      this.x = x;
      this.y = y;
      this.width = width;
      this.height = height;
   }

   public void dispose()
   {
      shadowSceneBatch.dispose();
   }

   public ArrayList<GDXPointLight> getPointLights()
   {
      return pointLights;
   }

   public ArrayList<GDXDirectionalLight> getDirectionalLights()
   {
      return directionalLights;
   }

   public void setAmbientLight(float ambientLight)
   {
      this.ambientLight = ambientLight;
   }

   public ModelBatch getShadowSceneBatch()
   {
      return shadowSceneBatch;
   }
}
