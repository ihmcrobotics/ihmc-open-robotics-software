package us.ihmc.gdx.simulation;

import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.glutils.*;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.utils.BufferUtils;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import imgui.internal.ImGui;
import imgui.type.ImFloat;
import org.apache.commons.lang3.tuple.Pair;
import org.lwjgl.opengl.GL41;
import org.lwjgl.opengl.GL43;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiVideoPanel;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.shader.GDXShader;
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
   private boolean depthEnabled = true;

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
   private int pointCloudBufferId;

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

      Pair<String, String> shaderStrings = GDXTools.loadCombinedShader(getClass().getName().replace(".", "/") + ".glsl");
      String vertexShader = shaderStrings.getLeft();
      String fragmentShader = shaderStrings.getRight();
      modelBatch = new ModelBatch(vertexShader, fragmentShader);

      SensorFrameBufferBuilder frameBufferBuilder = new SensorFrameBufferBuilder(imageWidth, imageHeight);
      frameBufferBuilder.addColorTextureAttachment(GL41.GL_RGBA8, GL41.GL_RGBA, GL41.GL_UNSIGNED_BYTE);
      frameBufferBuilder.addDepthTextureAttachment(GL41.GL_DEPTH_COMPONENT32F, GL41.GL_FLOAT);
      frameBuffer = frameBufferBuilder.build();

      pointCloudBufferId = GL41.glGenBuffers();
      GL41.glBufferData(pointCloudBufferId, imageWidth * imageHeight * 32, GL41.GL_DYNAMIC_COPY);

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
      render(sceneManager, null, null, 0.01f);
   }

   public void render(GDX3DSceneManager sceneManager, FloatBuffer pointCloudBufferToPack, Color userPointColor, float pointSize)
   {
      boolean updateThisTick = throttleTimer.isExpired(updatePeriod);
      if (updateThisTick)
         throttleTimer.reset();
      else
         return;

      frameBuffer.begin();
      GL41.glBindBuffer(GL43.GL_SHADER_STORAGE_BUFFER, pointCloudBufferId);

      float clear = 0.0f;
      GL41.glClearColor(clear, clear, clear, 1.0f);
      GL41.glClear(GL41.GL_COLOR_BUFFER_BIT | GL41.GL_DEPTH_BUFFER_BIT);

      viewport.update(imageWidth, imageHeight);
      modelBatch.begin(camera);
      GL41.glViewport(0, 0, imageWidth, imageHeight);

      sceneManager.renderExternalBatch(modelBatch, GDXSceneLevel.REAL_ENVIRONMENT);

      modelBatch.end();

      if (depthEnabled)
      {
         GL41.glPixelStorei(GL41.GL_UNPACK_ALIGNMENT, 4); // to read floats
         rawDepthByteBuffer.rewind();
         GL41.glReadPixels(0, 0, imageWidth, imageHeight, GL41.GL_DEPTH_COMPONENT, GL41.GL_FLOAT, rawDepthByteBuffer);
         GL41.glPixelStorei(GL41.GL_UNPACK_ALIGNMENT, 1); // undo what we did
      }
      //      rawColorByteBuffer.rewind();
      GL41.glPixelStorei(GL41.GL_UNPACK_ALIGNMENT, 4); // to read ints
      rawColorIntBuffer.rewind();
      GL41.glReadBuffer(GL41.GL_COLOR_ATTACHMENT0);
      GL41.glReadPixels(0, 0, imageWidth, imageHeight, GL41.GL_RGBA, GL41.GL_UNSIGNED_INT_8_8_8_8, rawColorIntBuffer);
      GL41.glPixelStorei(GL41.GL_UNPACK_ALIGNMENT, 1); // undo what we did

      frameBuffer.end();

      points.clear();
      colors.clear();

      float twoXCameraFarNear = 2.0f * camera.near * camera.far;
      float farPlusNear = camera.far + camera.near;
      float farMinusNear = camera.far - camera.near;
      rawDepthFloatBuffer.rewind();
      eyeDepthMetersBuffer.rewind();
      rawColorIntBuffer.rewind();
      if (depthEnabled)
      {
         for (int y = 0; y < imageHeight; y++)
         {
            for (int x = 0; x < imageWidth; x++)
            {
               float rawDepthReading = rawDepthFloatBuffer.get(); // 0.0 to 1.0
               int rawColorReading = rawColorIntBuffer.get();
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

                  if (pointCloudBufferToPack != null)
                  {
                     pointCloudBufferToPack.put(point.getX32());
                     pointCloudBufferToPack.put(point.getY32());
                     pointCloudBufferToPack.put(point.getZ32());

                     if (userPointColor != null)
                     {
                        pointCloudBufferToPack.put(userPointColor.r);
                        pointCloudBufferToPack.put(userPointColor.g);
                        pointCloudBufferToPack.put(userPointColor.b);
                        pointCloudBufferToPack.put(userPointColor.a);
                     }
                     else
                     {
                        pointCloudBufferToPack.put(((rawColorReading & 0xff000000) >>> 24) / 255f);
                        pointCloudBufferToPack.put(((rawColorReading & 0x00ff0000) >>> 16) / 255f);
                        pointCloudBufferToPack.put(((rawColorReading & 0x0000ff00) >>> 8) / 255f);
                        pointCloudBufferToPack.put(((rawColorReading & 0x000000ff)) / 255f);
                     }
                     pointCloudBufferToPack.put(pointSize);
                  }

                  colors.add(rawColorReading);
               }
            }
         }

         if (depthPanel.getIsShowing().get())
            depthWindowTexture.draw(depthWindowPixmap, 0, 0);
      }

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

   public ByteBuffer getColorRGBA8Buffer()
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

   public void setDepthEnabled(boolean depthEnabled)
   {
      this.depthEnabled = depthEnabled;
   }
}
