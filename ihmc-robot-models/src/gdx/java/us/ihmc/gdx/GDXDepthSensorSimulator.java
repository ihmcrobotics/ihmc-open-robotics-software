package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.PerspectiveCamera;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.g3d.shaders.DepthShader;
import com.badlogic.gdx.graphics.g3d.utils.DepthShaderProvider;
import com.badlogic.gdx.graphics.glutils.FrameBuffer;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;

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
   private FrameBuffer frameBuffer;

   private Point3D32[] points;

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

      frameBuffer = new FrameBuffer(Pixmap.Format.Intensity, (int) viewportWidth, (int) viewportHeight, true, true);

   }

   public void render(GDX3DApplication gdx3DApplication)
   {
      frameBuffer.begin();
      gdx3DApplication.glClearGrayscale();

      viewport.update((int) viewportWidth, (int) viewportHeight);
      modelBatch.begin(camera);

      Gdx.gl.glViewport(0, 0, (int) viewportWidth, (int) viewportHeight);

      gdx3DApplication.renderRegisteredObjectsWithEnvironment(modelBatch);

      modelBatch.end();
      frameBuffer.end();

      points = new Point3D32[(int) viewportWidth * (int) viewportHeight];

      Pixmap pixmap = frameBuffer.getColorBufferTexture().getTextureData().consumePixmap();
      for (int x = 0; x < viewportWidth; x++)
      {
         for (int y = 0; y < viewportHeight; y++)
         {
            int color = pixmap.getPixel(x, y);
            float depthReading = new Color(color).r;
            Vector3 worldRangePoint = viewport.unproject(new Vector3(x, y, depthReading));
            points[x * (int) viewportWidth + (int) viewportHeight] = new Point3D32(worldRangePoint.x, worldRangePoint.y, worldRangePoint.z);
         }
      }

//      frameBuffer.getColorBufferTexture().getTextureData().consumePixmap().getPixels().

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

   public Point3D32[] getPoints()
   {
      return points;
   }
}
