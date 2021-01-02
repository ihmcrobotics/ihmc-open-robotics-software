package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.PerspectiveCamera;
import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.g3d.shaders.DepthShader;
import com.badlogic.gdx.graphics.g3d.utils.DepthShaderProvider;
import com.badlogic.gdx.utils.viewport.ScreenViewport;

/**
 * Lidar could be simulated with a viewportHeight of 1 and rotating the camera
 */
public class GDXDepthSensorSimulator
{
   private float fieldOfViewY;
   private float viewportWidth;
   private float viewportHeight;
   private PerspectiveCamera camera;
   private int framebufferId;
   private DepthShaderProvider depthShaderProvider;

   private ModelBatch modelBatch;
   private int depthTextureId;
   private ScreenViewport viewport;

   public GDXDepthSensorSimulator(float fieldOfViewY, float viewportWidth, float viewportHeight)
   {
      this.fieldOfViewY = fieldOfViewY;
      this.viewportWidth = viewportWidth;
      this.viewportHeight = viewportHeight;
   }

   public void create()
   {
      camera = new PerspectiveCamera(fieldOfViewY, viewportWidth, viewportHeight);
      viewport = new ScreenViewport(camera);

      DepthShader.Config depthShaderConfig = new DepthShader.Config();
//      depthShaderConfig.numDirectionalLights = 0;
//      depthShaderConfig.numPointLights = 0;
//      depthShaderConfig.numSpotLights = 0;
//      depthShaderConfig.numBones = 0;
      depthShaderConfig.defaultCullFace = -1;
      depthShaderConfig.defaultAlphaTest = 0.5f;
      depthShaderConfig.vertexShader = Gdx.files.classpath("depthsensor.vertex.glsl").readString();
      depthShaderConfig.fragmentShader = Gdx.files.classpath("depthsensor.fragment.glsl").readString();
      //      config.depthBufferOnly = true;
      depthShaderProvider = new DepthShaderProvider(depthShaderConfig);

      modelBatch = new ModelBatch(depthShaderProvider);

      framebufferId = Gdx.gl.glGenFramebuffer();

      depthTextureId = Gdx.gl.glGenTexture();
   }

   public void render(GDX3DApplication gdx3DApplication)
   {
      gdx3DApplication.glClearGrayscale();

      viewport.update((int) viewportWidth, (int) viewportHeight);
      modelBatch.begin(camera);

      Gdx.gl.glBindFramebuffer(GL20.GL_FRAMEBUFFER, framebufferId);
      Gdx.gl.glEnable(GL20.GL_DEPTH_TEST);
      Gdx.gl.glBindTexture(framebufferId, depthTextureId);
      Gdx.gl.glFramebufferTexture2D(framebufferId, GL20.GL_DEPTH_ATTACHMENT, 0, depthTextureId, 0);

      Gdx.gl.glViewport(0, 0, (int) camera.viewportWidth, (int) camera.viewportHeight);

      gdx3DApplication.renderRegisteredObjectsWithEnvironment(modelBatch);

      modelBatch.end();

      // bind original framebuffer?
   }

   public void dispose()
   {
      Gdx.gl.glDeleteFramebuffer(framebufferId);
      Gdx.gl.glDeleteTexture(depthTextureId);
   }

   public PerspectiveCamera getCamera()
   {
      return camera;
   }
}
