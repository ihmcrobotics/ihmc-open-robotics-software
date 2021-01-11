package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.g3d.shaders.DepthShader;
import com.badlogic.gdx.graphics.g3d.utils.DepthShaderProvider;
import com.badlogic.gdx.graphics.glutils.FloatFrameBuffer;
import com.badlogic.gdx.graphics.glutils.FrameBuffer;
import com.badlogic.gdx.graphics.glutils.GLFrameBuffer;
import com.badlogic.gdx.graphics.glutils.GLOnlyTextureData;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.utils.ScreenUtils;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;

import java.nio.ByteBuffer;
import java.util.Random;

import static com.badlogic.gdx.graphics.GL20.GL_DEPTH_COMPONENT16;

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
   private FloatFrameBuffer floatframeBuffer;

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
//      depthShaderConfig.vertexShader = Gdx.files.classpath("depthsensor.vertex.glsl").readString();
//      depthShaderConfig.fragmentShader = Gdx.files.classpath("depthsensor.fragment.glsl").readString();
      //      config.depthBufferOnly = true;
      depthShaderProvider = new DepthShaderProvider(depthShaderConfig);

      modelBatch = new ModelBatch(depthShaderProvider);
//      modelBatch = new ModelBatch();

      boolean hasDepth = true;
      boolean hasStencil = true;
      frameBuffer = new FrameBuffer(Pixmap.Format.RGBA8888, (int) viewportWidth, (int) viewportHeight, hasDepth, hasStencil);
//      GLFrameBuffer.FloatFrameBufferBuilder depthBufferBuilder = new GLFrameBuffer.FloatFrameBufferBuilder((int) viewportWidth, (int) viewportHeight);
//      frameBuffer = new FloatFrameBuffer()FrameBuffer(depthBufferBuilder);
      floatframeBuffer = new FloatFrameBuffer((int) viewportWidth, (int) viewportHeight, true);
   }

   public void render(GDX3DApplication gdx3DApplication)
   {
      frameBuffer.begin();
      floatframeBuffer.begin();
      gdx3DApplication.glClearGrayscale();

      Random random = new Random();
//      viewport.getCamera().translate(-random.nextFloat(), -random.nextFloat(), -random.nextFloat());
//      viewport.getCamera().position

      viewport.update((int) viewportWidth, (int) viewportHeight);
      modelBatch.begin(camera);

      Gdx.gl.glViewport(0, 0, (int) viewportWidth, (int) viewportHeight);

      gdx3DApplication.renderRegisteredObjectsWithEnvironment(modelBatch);

      modelBatch.end();

      int width = frameBuffer.getWidth();
      int height = frameBuffer.getHeight();
//      frameBuffer.getDepthBufferHandle();
      Pixmap pixmap = ScreenUtils.getFrameBufferPixmap(0, 0, width, height);

//      ByteBuffer underlyingBuffer = frameBuffer.getColorBufferTexture().getTextureData().consumePixmap().getPixels();
//      Gdx.gl.glReadPixels(0, 0, width, height, GL_DEPTH_COMPONENT16, GL32.GL_HIGH_FLOAT, underlyingBuffer);

      frameBuffer.end();
      floatframeBuffer.end();

      points = new Point3D32[width * height];


//            GLOnlyTextureData textureData = (GLOnlyTextureData) frameBuffer.getColorBufferTexture().getTextureData();
//      Pixmap pixmap = textureData.consumePixmap();
      for (int y = 0; y < height; y++)
      {
         for (int x = 0; x < width; x++)
         {
            int color = pixmap.getPixel(x, y);
            float depthReading = new Color(color).r;
            Vector3 worldRangePoint = viewport.unproject(new Vector3(x, y, depthReading));
            int arrayIndex = y * width + x;
            int arrayLength = width * height;
            if (arrayIndex < arrayLength)
            {
               points[arrayIndex] = new Point3D32(worldRangePoint.x, worldRangePoint.y, worldRangePoint.z);
               points[arrayIndex].addZ(random.nextDouble() * 0.007);
            }
            else
            {
               System.out.println("why");
            }
         }
      }
      pixmap.dispose();

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
