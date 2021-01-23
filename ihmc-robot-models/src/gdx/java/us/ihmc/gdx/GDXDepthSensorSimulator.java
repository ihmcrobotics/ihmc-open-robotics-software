package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.g3d.shaders.DepthShader;
import com.badlogic.gdx.graphics.g3d.utils.DepthShaderProvider;
import com.badlogic.gdx.graphics.glutils.*;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.utils.BufferUtils;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import imgui.internal.ImGui;
import org.lwjgl.opengl.GL32;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.tools.GDXTools;

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.nio.ShortBuffer;
import java.util.Random;

/**
 * Lidar could be simulated with a viewportHeight of 1 and rotating the camera
 */
public class GDXDepthSensorSimulator
{
   private final String depthWindowName = "Depth";
   private final String colorWindowName = "Color";

   private final Random random = new Random();

   private final float fieldOfViewY;
   private final int imageWidth;
   private final int imageHeight;
   private final float minRange;
   private final float maxRange;
   private final Vector3 depthPoint = new Vector3();

   private PerspectiveCamera camera;
   private DepthShaderProvider depthShaderProvider;
   private ModelBatch modelBatch;
   private ScreenViewport viewport;
   private SensorFrameBuffer frameBuffer;
   private Pixmap pixmap;
   private RecyclingArrayList<Point3D32> points;

   private Pixmap depthWindowPixmap;
   private Texture depthWindowTexture;
   private Pixmap colorWindowPixmap;
   private Texture colorWindowTexture;

   private Matrix4 projection = new Matrix4();
   private int depthBufferHandle;
   private ByteBuffer depthByteBuffer;
   private Pixmap renderColorPixmap;
   private Pixmap renderDepthPixmap;
   private FloatTextureData depthTextureData;

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
//      depthShaderConfig.defaultCullFace = 0;
//      depthShaderConfig.depthBufferOnly = false;
//      depthShaderConfig.defaultAlphaTest = 1.0f;
//      depthShaderConfig.vertexShader = Gdx.files.classpath("depthsensor.vertex.glsl").readString();
//      depthShaderConfig.fragmentShader = Gdx.files.classpath("depthsensor.fragment.glsl").readString();
      depthShaderProvider = new DepthShaderProvider(depthShaderConfig);

//      modelBatch = new ModelBatch(depthShaderProvider);
      modelBatch = new ModelBatch();

//      boolean hasDepth = true;
//      frameBuffer = new FrameBuffer(Pixmap.Format.RGBA8888, imageWidth, imageHeight, hasDepth);

      SensorFrameBufferBuilder frameBufferBuilder = new SensorFrameBufferBuilder(imageWidth, imageHeight);
      frameBufferBuilder.addBasicColorTextureAttachment(Pixmap.Format.RGBA8888);
      frameBufferBuilder.addDepthTextureAttachment(GL30.GL_DEPTH_COMPONENT32F, GL30.GL_FLOAT);
      frameBuffer = frameBufferBuilder.build();

      renderColorPixmap = frameBuffer.getColorPixmap();
      depthTextureData = frameBuffer.getDepthTextureData();

//      renderColorPixmap = frameBuffer.getTextureAttachments().get(0).getTextureData().consumePixmap();
//      renderDepthPixmap = frameBuffer.getTextureAttachments().get(1).getTextureData().consumePixmap();

      int glDepthComponent16 = GL20.GL_DEPTH_COMPONENT16;

      depthBufferHandle = frameBuffer.getDepthBufferHandle();
      depthByteBuffer = BufferUtils.newByteBuffer(imageWidth * imageHeight * 4);

//      pixmap = new Pixmap(imageWidth, imageHeight, Pixmap.Format.RGBA8888);

      depthWindowPixmap = new Pixmap(imageWidth, imageHeight, Pixmap.Format.RGBA8888);
      depthWindowTexture = new Texture(new PixmapTextureData(depthWindowPixmap, null, false, false));

      colorWindowPixmap = new Pixmap(imageWidth, imageHeight, Pixmap.Format.RGBA8888);
      colorWindowTexture = new Texture(new PixmapTextureData(colorWindowPixmap, null, false, false));

      points = new RecyclingArrayList<>(imageWidth * imageHeight, Point3D32::new);
   }

   public void render(GDX3DSceneManager sceneManager)
   {
      frameBuffer.begin();

      float clear = 0.0f;
      Gdx.gl.glClearColor(clear, clear, clear, 1.0f);
      Gdx.gl.glClear(GL32.GL_COLOR_BUFFER_BIT | GL32.GL_DEPTH_BUFFER_BIT);

      viewport.update(imageWidth, imageHeight);
      modelBatch.begin(camera);
      Gdx.gl.glViewport(0, 0, imageWidth, imageHeight);

      sceneManager.renderRegisteredObjectsWithEnvironment(modelBatch, GDXSceneLevel.REAL_ENVIRONMENT);

      modelBatch.end();

//      Gdx.gl.glPixelStorei(GL20.GL_PACK_ALIGNMENT, 1);
//      ByteBuffer pixels = pixmap.getPixels();
//      pixels.rewind();
//      Gdx.gl.glReadPixels(0, 0, imageWidth, imageHeight, GL20.GL_RGBA, GL20.GL_UNSIGNED_BYTE, pixels);
//
//      Gdx.gl20.glBindRenderbuffer(GL20.GL_RENDERBUFFER, depthBufferHandle);
//      depthByteBuffer.rewind();
//      Gdx.gl.glReadPixels(0, 0, imageWidth, imageHeight, GL20.GL_LUMINANCE_ALPHA, GL20.GL_UNSIGNED_BYTE, depthByteBuffer);
//      ShortBuffer shortBuffer = depthByteBuffer.asShortBuffer();

//      Gdx.gl.glTexImage2D(target, 0, internalFormat, width, height, 0, format, GL20.GL_FLOAT, buffer);
      Gdx.gl.glPixelStorei(GL20.GL_PACK_ALIGNMENT, 4);
      depthByteBuffer.rewind();
      Gdx.gl.glReadPixels(0, 0, imageWidth, imageHeight, GL30.GL_DEPTH_COMPONENT, GL30.GL_FLOAT, depthByteBuffer);

      frameBuffer.end();

//      frameBuffer.getDepthTexture().bind();
//      Gdx.gl.glReadPixels(0, 0, imageWidth, imageHeight, GL30.GL_DEPTH_COMPONENT, GL20.GL_UNSIGNED_BYTE, depthTextureData.getBuffer());
//      depthTextureData.getBuffer().rewind();
//      depthTextureData.consumeCustomData(frameBuffer.getDepthTexture().glTarget);
//      SensorFrameBuffer.unbind();

      points.clear();

//      depthTextureData.getBuffer().rewind();
      depthByteBuffer.rewind();
      FloatBuffer depthFloatBuffer = depthByteBuffer.asFloatBuffer();
      double averageDepth = 0.0;
      for (int y = 0; y < imageHeight; y++)
      {
         for (int x = 0; x < imageWidth; x++)
         {
//            int encodedDepthValue = pixmap.getPixel(x, y);
            int encodedDepthValue = renderColorPixmap.getPixel(x, y);

            float depthReading = ((encodedDepthValue & 0xff000000) >>> 24) / 255.0f;
            depthReading += ((encodedDepthValue & 0x00ff0000) >>> 16) / 65025.0f;
            depthReading += ((encodedDepthValue & 0x0000ff00) >>> 8) / 16581375.0f;
            depthReading += ((encodedDepthValue & 0x000000ff)) / 4294967296.0f;


//            depthReading = shortBuffer.get(y * imageWidth + x);
//            depthReading = -shortBuffer.get() / 100.0f;
//            int depthValue = renderDepthPixmap.getPixel(x, y);
//            short shortValue = shortBuffer.get();
//            int unsignedInt = Short.toUnsignedInt(shortValue);
//            depthReading = Float.intBitsToFloat(depthValue);
//            depthReading = depthTextureData.getBuffer().get();
            depthReading = depthFloatBuffer.get();

            if (x == 200 && y == 300)
            {
               averageDepth += depthReading;
            }
            else
            {
               averageDepth += depthReading;
            }
//
            float clippedDepth = (float) MathTools.clamp(depthReading, camera.near, camera.far);
            float pastNear = clippedDepth - camera.near;
            float worldRange = camera.far - camera.near;
            float colorRange = 1.0f;
            float grayscale = pastNear * colorRange / worldRange;
            grayscale = depthReading * colorRange / camera.far;
            int flippedY = imageHeight - y;

//            colorWindowPixmap.drawPixel(x, flippedY, encodedDepthValue);
            depthWindowPixmap.drawPixel(x, flippedY, Color.rgba8888(grayscale, grayscale, grayscale, 1.0f));

            if (depthReading > camera.near && depthReading < camera.far)
            {
               float halfFieldOfViewY = (float) Math.toRadians(fieldOfViewY) / 2.0f;
               float halfFieldOfViewX = halfFieldOfViewY * (float) imageHeight / (float) imageWidth;
               float percentToXLimit = (2.0f * x) / imageWidth - 1.0f;
               float percentToYLimit = (2.0f * flippedY) / imageHeight - 1.0f;
               float angleX = percentToXLimit * halfFieldOfViewX;
               float angleY = percentToYLimit * halfFieldOfViewY;

               float alpha = (float) Math.sqrt(angleX * angleX + angleY * angleY);
               float viewZ = depthReading * (float) Math.cos(alpha);
               float viewX = viewZ * (float) Math.tan(angleX);
               float viewY = viewZ * (float) Math.tan(angleY);
               depthPoint.set(viewX, viewY, viewZ);

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

      averageDepth /= imageWidth * imageHeight;
//      System.out.println("Average depth: " + averageDepth);

//      frameBuffer.getColorTexture().draw(colorWindowPixmap, 0, 0);
//      frameBuffer.getDepthTexture().draw(depthWindowPixmap, 0, 0);
      depthWindowTexture.draw(depthWindowPixmap, 0, 0);
   }

   public void renderImGuiDepthWindow(GDX3DSceneManager sceneManager)
   {
      ImGui.begin(depthWindowName);

      float posX = ImGui.getWindowPosX();
      float posY = ImGui.getWindowPosY();
      float sizeX = ImGui.getWindowSizeX();
      float sizeY = ImGui.getWindowSizeY();

//      int textureId = frameBuffer.getDepthTexture().getTextureObjectHandle();
      int textureId = depthWindowTexture.getTextureObjectHandle();

//      ImGui.getWindowDrawList().addImage(textureId, posX, posY + sizeY, posX + sizeX, posY);
      ImGui.getWindowDrawList().addImage(textureId, posX, posY, posX + sizeX, posY + sizeY);

      sceneManager.getCamera3D().addInputExclusionBox(ImGuiTools.windowBoundingBox());
      ImGui.end();
   }

   public void renderImGuiColorWindow(GDX3DSceneManager sceneManager)
   {
      ImGui.begin(colorWindowName);

      float posX = ImGui.getWindowPosX();
      float posY = ImGui.getWindowPosY();
      float sizeX = ImGui.getWindowSizeX();
      float sizeY = ImGui.getWindowSizeY();

      int textureId = frameBuffer.getColorTexture().getTextureObjectHandle();

      ImGui.getWindowDrawList().addImage(textureId, posX, posY + sizeY, posX + sizeX, posY);

      sceneManager.getCamera3D().addInputExclusionBox(ImGuiTools.windowBoundingBox());
      ImGui.end();
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

   public String getDepthWindowName()
   {
      return depthWindowName;
   }

   public String getColorWindowName()
   {
      return colorWindowName;
   }
}
