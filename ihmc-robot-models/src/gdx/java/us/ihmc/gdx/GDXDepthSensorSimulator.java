package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.g3d.shaders.DepthShader;
import com.badlogic.gdx.graphics.g3d.utils.DepthShaderProvider;
import com.badlogic.gdx.graphics.glutils.FrameBuffer;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import org.lwjgl.opengl.GL32;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.tools.GDXTools;

import java.nio.ByteBuffer;
import java.util.Random;

/**
 * Lidar could be simulated with a viewportHeight of 1 and rotating the camera
 */
public class GDXDepthSensorSimulator
{
   private final Random random = new Random();

   private final float fieldOfViewY;
   private final int imageWidth;
   private final int imageHeight;
   private final float minRange;
   private final float maxRange;
   private final Color gdxColor = new Color();
   private final Vector3 depthPoint = new Vector3();

   private PerspectiveCamera camera;
   private DepthShaderProvider depthShaderProvider;
   private ModelBatch modelBatch;
   private ScreenViewport viewport;
   private FrameBuffer frameBuffer;
   private Pixmap pixmap;
   private RecyclingArrayList<Point3D32> points;

   public GDXDepthSensorSimulator(double fieldOfViewY, int imageWidth, int imageHeight, double minRange, double maxRange)
   {
      this.fieldOfViewY = (float) fieldOfViewY;
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
      this.minRange = (float) minRange;
      this.maxRange = (float) maxRange;
   }

   public void create()
   {
      camera = new PerspectiveCamera(fieldOfViewY, imageWidth, imageHeight);
      camera.near = minRange;
      camera.far = maxRange;
      viewport = new ScreenViewport(camera);

      DepthShader.Config depthShaderConfig = new DepthShader.Config();
      depthShaderConfig.defaultCullFace = GL20.GL_BACK;
      depthShaderConfig.vertexShader = Gdx.files.classpath("depthsensor.vertex.glsl").readString();
      depthShaderConfig.fragmentShader = Gdx.files.classpath("depthsensor.fragment.glsl").readString();
      depthShaderProvider = new DepthShaderProvider(depthShaderConfig);

      modelBatch = new ModelBatch(depthShaderProvider);

      boolean hasDepth = true;
      frameBuffer = new FrameBuffer(Pixmap.Format.RGBA8888, imageWidth, imageHeight, hasDepth);
      pixmap = new Pixmap(imageWidth, imageHeight, Pixmap.Format.RGBA8888);

      points = new RecyclingArrayList<>(imageWidth * imageHeight, Point3D32::new);
   }

   public void render(GDX3DSceneManager gdx3DSceneManager)
   {
      frameBuffer.begin();

      float clear = 0.0f;
      Gdx.gl.glClearColor(clear, clear, clear, 1.0f);
      Gdx.gl.glClear(GL32.GL_COLOR_BUFFER_BIT | GL32.GL_DEPTH_BUFFER_BIT);

      viewport.update(imageWidth, imageHeight);
      modelBatch.begin(camera);
      Gdx.gl.glViewport(0, 0, imageWidth, imageHeight);

      gdx3DSceneManager.renderRegisteredObjectsWithEnvironment(modelBatch, GDXSceneLevel.REAL_ENVIRONMENT);

      modelBatch.end();

      Gdx.gl.glPixelStorei(GL20.GL_PACK_ALIGNMENT, 1);
      ByteBuffer pixels = pixmap.getPixels();
      pixels.rewind();
      Gdx.gl.glReadPixels(0, 0, imageWidth, imageHeight, GL20.GL_RGBA, GL20.GL_UNSIGNED_BYTE, pixels);

      frameBuffer.end();

      points.clear();
      for (int y = 0; y < imageHeight; y++)
      {
         for (int x = 0; x < imageWidth; x++)
         {
            int color = pixmap.getPixel(x, y);
            gdxColor.r = ((color & 0xff000000) >>> 24) / 255f;
            gdxColor.g = ((color & 0x00ff0000) >>> 16) / 255f;
            gdxColor.b = ((color & 0x0000ff00) >>> 8) / 255f;
            gdxColor.a = ((color & 0x000000ff)) / 255f;
            float depthReading = gdxColor.r;
            depthReading += gdxColor.g / 256.0;
            depthReading += gdxColor.b / 65536.0;
            depthReading += gdxColor.a / 16777216.0;

            if (depthReading > camera.near)
            {
               depthPoint.x = (2.0f * x) / imageWidth - 1.0f;
               depthPoint.y = (2.0f * y) / imageHeight - 1.0f;
               depthPoint.z = 2.0f * depthReading - 1.0f;
               depthPoint.prj(camera.invProjectionView);

               Point3D32 point = points.add();
               GDXTools.toEuclid(depthPoint, point);
               point.addZ(random.nextDouble() * 0.007); // add some noise
            }
         }
      }
   }

   public void dispose()
   {
      frameBuffer.dispose();
      depthShaderProvider.dispose();
      pixmap.dispose();
   }

   public void setCameraWorldTransform(Matrix4 worldTransform)
   {
      camera.position.setZero();
      camera.up.set(0.0f, 0.0f, 1.0f);
      camera.direction.set(1.0f, 0.0f, 0.0f);
      camera.transform(worldTransform);
   }

   public PerspectiveCamera getCamera()
   {
      return camera;
   }

   public RecyclingArrayList<Point3D32> getPoints()
   {
      return points;
   }
}
