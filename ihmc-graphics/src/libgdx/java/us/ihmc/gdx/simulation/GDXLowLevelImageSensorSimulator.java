package us.ihmc.gdx.simulation;

import com.badlogic.gdx.Gdx;
import org.lwjgl.opengl.GL41;
import com.badlogic.gdx.graphics.PerspectiveCamera;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.glutils.SensorFrameBuffer;
import com.badlogic.gdx.graphics.glutils.SensorFrameBufferBuilder;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.BufferUtils;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import org.lwjgl.opengl.GL41;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiVideoPanel;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.tools.Timer;
import us.ihmc.tools.UnitConversions;

import java.nio.ByteBuffer;
import java.nio.IntBuffer;

/**
 * Lidar could be simulated with a viewportHeight of 1 and rotating the camera
 */
public class GDXLowLevelImageSensorSimulator
{
   private final String colorWindowName;

   private final float fieldOfViewY;
   private final int imageWidth;
   private final int imageHeight;
   private final float minRange;
   private final float maxRange;
   private final double updatePeriod;
   private final Timer throttleTimer = new Timer();

   private PerspectiveCamera camera;
   private ModelBatch modelBatch;
   private ScreenViewport viewport;
   private SensorFrameBuffer frameBuffer;

   private final ImGuiVideoPanel colorPanel;

   private ByteBuffer rawColorByteBuffer;
   private IntBuffer rawColorIntBuffer;

   public GDXLowLevelImageSensorSimulator(String sensorName, double fieldOfViewY, int imageWidth, int imageHeight, double minRange, double maxRange)
   {
      colorWindowName = ImGuiTools.uniqueLabel(sensorName + " Color");
      this.fieldOfViewY = (float) fieldOfViewY;
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
      this.minRange = (float) minRange;
      this.maxRange = (float) maxRange;
      this.updatePeriod = UnitConversions.hertzToSeconds(30.0);
      colorPanel = new ImGuiVideoPanel(colorWindowName, true);
   }

   public void create()
   {
      throttleTimer.reset();

      camera = new PerspectiveCamera(fieldOfViewY, imageWidth, imageHeight);
      camera.near = minRange;
      camera.far = maxRange * 2.0f; // should render camera farther
      viewport = new ScreenViewport(camera);

//      DefaultShader.Config config = new DefaultShader.Config();
//      config.defaultCullFace = GL41.GL_BACK;
//      DefaultShaderProvider defaultShaderProvider = new DefaultShaderProvider(config);
//      modelBatch = new ModelBatch(defaultShaderProvider);
      modelBatch = new ModelBatch();

      SensorFrameBufferBuilder frameBufferBuilder = new SensorFrameBufferBuilder(imageWidth, imageHeight);
      frameBufferBuilder.addBasicColorTextureAttachment(Pixmap.Format.RGBA8888);
      frameBufferBuilder.addDepthTextureAttachment(GL41.GL_DEPTH_COMPONENT32F, GL41.GL_FLOAT);
      frameBuffer = frameBufferBuilder.build();

      rawColorByteBuffer = BufferUtils.newByteBuffer(imageWidth * imageHeight * 4);
      rawColorIntBuffer = rawColorByteBuffer.asIntBuffer();

      colorPanel.setTexture(frameBuffer.getColorTexture());
   }

   public void render(GDX3DSceneManager sceneManager)
   {
      boolean updateThisTick = throttleTimer.isExpired(updatePeriod);
      if (updateThisTick)
         throttleTimer.reset();
      else
         return;

      frameBuffer.begin();

      float clear = 0.0f;
      GL41.glClearColor(clear, clear, clear, 1.0f);
//      GL41.glClear(GL41.GL_COLOR_BUFFER_BIT | GL41.GL_DEPTH_BUFFER_BIT);
      GL41.glClear(GL41.GL_COLOR_BUFFER_BIT);

      viewport.update(imageWidth, imageHeight);
      modelBatch.begin(camera);
      GL41.glViewport(0, 0, imageWidth, imageHeight);

      sceneManager.renderExternalBatch(modelBatch, GDXSceneLevel.REAL_ENVIRONMENT);

      modelBatch.end();

      GL41.glPixelStorei(GL41.GL_PACK_ALIGNMENT, 1);
      rawColorByteBuffer.rewind();
      GL41.glReadPixels(0, 0, imageWidth, imageHeight, GL41.GL_RGBA, GL41.GL_UNSIGNED_BYTE, rawColorByteBuffer);

      frameBuffer.end();
   }

   public void dispose()
   {
      frameBuffer.dispose();
      modelBatch.dispose();
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

   public IntBuffer getColorRGB8Buffer()
   {
      return rawColorIntBuffer;
   }

   public ByteBuffer getRawColorByteBuffer()
   {
      return rawColorByteBuffer;
   }

   public Pixmap getColorPixmap()
   {
      return frameBuffer.getColorPixmap();
   }

   public float getMaxRange()
   {
      return maxRange;
   }

   public ImGuiVideoPanel getColorPanel()
   {
      return colorPanel;
   }
}
