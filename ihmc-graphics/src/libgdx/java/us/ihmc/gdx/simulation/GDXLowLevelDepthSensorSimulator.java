package us.ihmc.gdx.simulation;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.glutils.*;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.utils.BufferUtils;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import imgui.internal.ImGui;
import imgui.type.ImFloat;
import org.lwjgl.opengl.GL32;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiVideoPanel;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.tools.Timer;
import us.ihmc.tools.UnitConversions;

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.ArrayList;
import java.util.Random;

/**
 * Lidar could be simulated with a viewportHeight of 1 and rotating the camera
 */
public class GDXLowLevelDepthSensorSimulator
{
   private final String depthWindowName;
   private final String colorWindowName;

   private final Random random = new Random();

   private final float fieldOfViewY;
   private final int imageWidth;
   private final int imageHeight;
   private final float minRange;
   private final float maxRange;
   private final double updatePeriod;
   private final Timer throttleTimer = new Timer();
   private final Vector3 depthPoint = new Vector3();
   private final Vector3D32 noiseVector = new Vector3D32();

   private PerspectiveCamera camera;
   private ModelBatch modelBatch;
   private ScreenViewport viewport;
   private SensorFrameBuffer frameBuffer;
   private RecyclingArrayList<Point3D32> points;
   private ArrayList<Integer> colors;
   private boolean colorsAreBeingUsed = true;

   private Pixmap depthWindowPixmap;
   private Texture depthWindowTexture;
   private final ImGuiVideoPanel depthPanel;
   private final ImGuiVideoPanel colorPanel;
   private float lowestValueSeen = -1.0f;
   private float highestValueSeen = -1.0f;

   private ByteBuffer rawDepthByteBuffer;
   private FloatBuffer rawDepthFloatBuffer;
   private FloatBuffer eyeDepthMetersBuffer;
   private ByteBuffer rawColorByteBuffer;
   private IntBuffer rawColorIntBuffer;

   private final ImFloat depthPitchTuner = new ImFloat(-0.027f);

   public GDXLowLevelDepthSensorSimulator(String sensorName, double fieldOfViewY, int imageWidth, int imageHeight, double minRange, double maxRange)
   {
      depthWindowName = ImGuiTools.uniqueLabel(sensorName + " Depth");
      colorWindowName = ImGuiTools.uniqueLabel(sensorName + " Color");
      this.fieldOfViewY = (float) fieldOfViewY;
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
      this.minRange = (float) minRange;
      this.maxRange = (float) maxRange;
      this.updatePeriod = UnitConversions.hertzToSeconds(30.0);

      depthPanel = new ImGuiVideoPanel(depthWindowName, false);
      colorPanel = new ImGuiVideoPanel(colorWindowName, true);
   }

   public void create()
   {
      throttleTimer.reset();

      camera = new PerspectiveCamera(fieldOfViewY, imageWidth, imageHeight);
      camera.near = minRange;
      camera.far = maxRange * 2.0f; // should render camera farther
      viewport = new ScreenViewport(camera);

      modelBatch = new ModelBatch();

      SensorFrameBufferBuilder frameBufferBuilder = new SensorFrameBufferBuilder(imageWidth, imageHeight);
      frameBufferBuilder.addBasicColorTextureAttachment(Pixmap.Format.RGBA8888);
      frameBufferBuilder.addDepthTextureAttachment(GL30.GL_DEPTH_COMPONENT32F, GL30.GL_FLOAT);
      frameBuffer = frameBufferBuilder.build();

      rawDepthByteBuffer = BufferUtils.newByteBuffer(imageWidth * imageHeight * 4);
      rawDepthFloatBuffer = rawDepthByteBuffer.asFloatBuffer();

      rawColorByteBuffer = BufferUtils.newByteBuffer(imageWidth * imageHeight * 4);
      rawColorIntBuffer = rawColorByteBuffer.asIntBuffer();

      eyeDepthMetersBuffer = BufferUtils.newFloatBuffer(imageWidth * imageHeight);

      depthWindowPixmap = new Pixmap(imageWidth, imageHeight, Pixmap.Format.RGBA8888);
      depthWindowTexture = new Texture(new PixmapTextureData(depthWindowPixmap, null, false, false));

      depthPanel.setTexture(depthWindowTexture);
      colorPanel.setTexture(frameBuffer.getColorTexture());

      points = new RecyclingArrayList<>(imageWidth * imageHeight, Point3D32::new);
      colors = new ArrayList<>(imageWidth * imageHeight);
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
      Gdx.gl.glClearColor(clear, clear, clear, 1.0f);
      Gdx.gl.glClear(GL32.GL_COLOR_BUFFER_BIT | GL32.GL_DEPTH_BUFFER_BIT);

      viewport.update(imageWidth, imageHeight);
      modelBatch.begin(camera);
      Gdx.gl.glViewport(0, 0, imageWidth, imageHeight);

      sceneManager.renderRegisteredObjectsWithEnvironment(modelBatch, GDXSceneLevel.REAL_ENVIRONMENT);

      modelBatch.end();

      Gdx.gl.glPixelStorei(GL20.GL_PACK_ALIGNMENT, 4);
      rawDepthByteBuffer.rewind();
      Gdx.gl.glReadPixels(0, 0, imageWidth, imageHeight, GL30.GL_DEPTH_COMPONENT, GL30.GL_FLOAT, rawDepthByteBuffer);
      Gdx.gl.glPixelStorei(GL20.GL_PACK_ALIGNMENT, 1);
      rawColorByteBuffer.rewind();
      Gdx.gl.glReadPixels(0, 0, imageWidth, imageHeight, GL30.GL_RGBA, GL30.GL_UNSIGNED_BYTE, rawColorByteBuffer);

      frameBuffer.end();

      points.clear();
      colors.clear();

      float twoXCameraFarNear = 2.0f * camera.near * camera.far;
      float farPlusNear = camera.far + camera.near;
      float farMinusNear = camera.far - camera.near;
      rawDepthFloatBuffer.rewind();
      eyeDepthMetersBuffer.rewind();
      for (int y = 0; y < imageHeight; y++)
      {
         for (int x = 0; x < imageWidth; x++)
         {
            float rawDepthReading = rawDepthFloatBuffer.get(); // 0.0 to 1.0
            float imageY = (2.0f * y) / imageHeight - 1.0f;

            // From "How to render depth linearly in modern OpenGL with gl_FragCoord.z in fragment shader?"
            // https://stackoverflow.com/a/45710371/1070333
            float normalizedDeviceCoordinateZ = 2.0f * rawDepthReading - 1.0f; // -1.0 to 1.0
            float eyeDepth = (twoXCameraFarNear / (farPlusNear - normalizedDeviceCoordinateZ * farMinusNear)); // in meters
            eyeDepth += imageY * depthPitchTuner.get();
            eyeDepthMetersBuffer.put(eyeDepth);

            if (depthPanel.getIsShowing().get())
            {
               if (highestValueSeen < 0 || eyeDepth > highestValueSeen)
                  highestValueSeen = eyeDepth;
               if (lowestValueSeen < 0 || eyeDepth < lowestValueSeen)
                  lowestValueSeen = eyeDepth;

               float colorRange = highestValueSeen - lowestValueSeen;
               float grayscale = (eyeDepth - lowestValueSeen) / colorRange;
               int flippedY = imageHeight - y;

               depthWindowPixmap.drawPixel(x, flippedY, Color.rgba8888(grayscale, grayscale, grayscale, 1.0f));
            }

            if (eyeDepth > camera.near && eyeDepth < maxRange)
            {
               depthPoint.x = (2.0f * x) / imageWidth - 1.0f;
               depthPoint.y = imageY;
               depthPoint.z = 2.0f * rawDepthReading - 1.0f;
               depthPoint.prj(camera.invProjectionView);

               Point3D32 point = points.add();
               GDXTools.toEuclid(depthPoint, point);

               GDXTools.toEuclid(camera.position, noiseVector);
               noiseVector.sub(point);
               noiseVector.normalize();
               noiseVector.scale((random.nextDouble() - 0.5) * 0.007);
               point.add(noiseVector);

               if (colorsAreBeingUsed)
                  colors.add(frameBuffer.getColorPixmap().getPixel(x, imageHeight - y)); // this is not working
            }
         }
      }

      if (depthPanel.getIsShowing().get())
         depthWindowTexture.draw(depthWindowPixmap, 0, 0);

      colorsAreBeingUsed = false;
   }

   public void renderTuningSliders()
   {
      ImGui.dragFloat(ImGuiTools.uniqueLabel(this, "Depth Pitch Tuner"), depthPitchTuner.getData(), 0.0001f, -0.05f, 0.05f);
   }

   public void dispose()
   {
      frameBuffer.dispose();
      depthWindowTexture.dispose();
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

   public FloatBuffer getEyeDepthMetersBuffer()
   {
      return eyeDepthMetersBuffer;
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

   public RecyclingArrayList<Point3D32> getPoints()
   {
      return points;
   }

   public ArrayList<Integer> getColors()
   {
      colorsAreBeingUsed = true;
      return colors;
   }

   public ImGuiVideoPanel getDepthPanel()
   {
      return depthPanel;
   }

   public ImGuiVideoPanel getColorPanel()
   {
      return colorPanel;
   }
}
