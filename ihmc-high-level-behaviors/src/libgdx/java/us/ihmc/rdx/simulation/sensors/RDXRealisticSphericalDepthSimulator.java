package us.ihmc.rdx.simulation.sensors;

import com.badlogic.gdx.graphics.PerspectiveCamera;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.glutils.SensorFrameBuffer;
import com.badlogic.gdx.graphics.glutils.SensorFrameBufferBuilder;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import org.apache.commons.lang3.tuple.Pair;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.rdx.sceneManager.RDX3DScene;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.DepthSensorShaderProvider;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.tools.UnitConversions;

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;

/**
 * Lidar could be simulated with a viewportHeight of 1 and rotating the camera
 */
public class RDXRealisticSphericalDepthSimulator
{
   private final int imageWidth;
   private final int imageHeight;
   private final int numberOfPoints;

   /**
    * Simulated camera that observes the current GDX Scene
    **/
   private PerspectiveCamera camera;
   private ScreenViewport viewport;

   private RigidBodyTransform transformToWorldFrame = new RigidBodyTransform();
   private ModelBatch modelBatch;
   private SensorFrameBuffer frameBuffer;
   private boolean depthEnabled = true;

   private BytedecoImage metersDepthImage;
   private boolean initialized = false;

   public RDXRealisticSphericalDepthSimulator(String sensorName,
                                              double fieldOfViewY,
                                              int imageWidth,
                                              int imageHeight,
                                              double minRange,
                                              double maxRange)
   {
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;

      numberOfPoints = imageWidth * imageHeight;
      metersDepthImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_16UC1);
   }

   public void create()
   {
      Pair<String, String> shaderStrings = LibGDXTools.loadCombinedShader(getClass().getName().replace(".", "/") + ".glsl");
      String vertexShader = shaderStrings.getLeft();
      String fragmentShader = shaderStrings.getRight();
      modelBatch = new ModelBatch(null, new DepthSensorShaderProvider(vertexShader, fragmentShader), null);

      SensorFrameBufferBuilder frameBufferBuilder = new SensorFrameBufferBuilder(imageWidth, imageHeight);
      frameBufferBuilder.addColorTextureAttachment(GL41.GL_RGBA8, GL41.GL_RGBA, GL41.GL_UNSIGNED_BYTE);
      frameBufferBuilder.addDepthTextureAttachment(GL41.GL_DEPTH_COMPONENT32F, GL41.GL_FLOAT);
      frameBufferBuilder.addColorTextureAttachment(GL41.GL_R32F, GL41.GL_RED, GL41.GL_FLOAT);
      frameBuffer = frameBufferBuilder.build();

      initialized = true;
   }

   public void render(RDX3DScene scene)
   {
      if (initialized)
      {

         // Render the lidar scan to the framebuffer
         frameBuffer.begin();

         // Clear the screen
         GL41.glClearColor(0, 0, 0, 1);
         GL41.glClear(GL41.GL_COLOR_BUFFER_BIT | GL41.GL_DEPTH_BUFFER_BIT);

         GL41.glViewport(0, 0, imageWidth, imageHeight);

         modelBatch.begin(camera);
         GL41.glViewport(0, 0, imageWidth, imageHeight);

         scene.renderExternalBatch(modelBatch, RDXSceneLevel.GROUND_TRUTH.SINGLETON_SET);

         modelBatch.end();

         GL41.glReadBuffer(GL41.GL_COLOR_ATTACHMENT0);
         GL41.glPixelStorei(GL41.GL_UNPACK_ALIGNMENT, 4); // to read ints

         metersDepthImage.getBackingDirectByteBuffer().rewind();

         GL41.glReadPixels(0, 0, imageWidth, imageHeight, GL41.GL_RED, GL41.GL_R16, metersDepthImage.getBackingDirectByteBuffer());
         GL41.glPixelStorei(GL41.GL_UNPACK_ALIGNMENT, 1); // undo what we did

         frameBuffer.end();

         // Get the depth buffer from the framebuffer
         Texture colorBufferTexture = frameBuffer.getColorBufferTexture();
         colorBufferTexture.getTextureData().prepare();
         int width = colorBufferTexture.getWidth();
         int height = colorBufferTexture.getHeight();
         int numChannels = 4; // RGBA
      }
   }

   //public void render(RDX3DScene scene, boolean colorBasedOnWorldZ, Color userPointColor, float pointSize)
   //{
   //   boolean updateThisTick = throttleTimer.isExpired(updatePeriod);
   //   if (updateThisTick)
   //      throttleTimer.reset();
   //   else
   //      return;
   //
   //   frameBuffer.begin();
   //
   //   float clear = 0.0f;
   //   GL41.glClearColor(clear, clear, clear, 1.0f);
   //   GL41.glClear(GL41.GL_COLOR_BUFFER_BIT | GL41.GL_DEPTH_BUFFER_BIT);
   //
   //   for (ScreenViewport viewport : viewports)
   //   {
   //      viewport.update(imageWidth, imageHeight);
   //   }
   //
   //   modelBatch.begin(camera);
   //   GL41.glViewport(0, 0, imageWidth, imageHeight);
   //
   //   scene.renderExternalBatch(modelBatch, RDXSceneLevel.GROUND_TRUTH.SINGLETON_SET);
   //
   //   modelBatch.end();
   //
   //   GL41.glReadBuffer(GL41.GL_COLOR_ATTACHMENT0);
   //   GL41.glPixelStorei(GL41.GL_UNPACK_ALIGNMENT, 4); // to read ints
   //   rgba8888ColorImage.getBackingDirectByteBuffer().rewind();
   //   // Careful, if loaded as type GL_UNSIGNED_INT_8_8_8_8, the bytes with be in native order, sometime AGBR (flipped)
   //   // See https://stackoverflow.com/questions/7786187/opengl-texture-upload-unsigned-byte-vs-unsigned-int-8-8-8-8
   //   GL41.glReadPixels(0, 0, imageWidth, imageHeight, GL41.GL_RGBA, GL41.GL_UNSIGNED_BYTE, rgba8888ColorImage.getBackingDirectByteBuffer());
   //   GL41.glPixelStorei(GL41.GL_UNPACK_ALIGNMENT, 1); // undo what we did
   //
   //   if (depthEnabled)
   //   {
   //      for (int i = 0; i< totalCameras; i++)
   //      {
   //         normalizedDeviceCoordDepthImages.get(i).getBackingDirectByteBuffer().rewind();
   //
   //         GL41.glReadBuffer(GL41.GL_COLOR_ATTACHMENT1);
   //         GL41.glPixelStorei(GL41.GL_UNPACK_ALIGNMENT, 4); // to read floats
   //         GL41.glReadPixels(0, 0, imageWidth, imageHeight, GL41.GL_RED, GL41.GL_FLOAT, normalizedDeviceCoordDepthImages.getBackingDirectByteBuffer());
   //         GL41.glPixelStorei(GL41.GL_UNPACK_ALIGNMENT, 1); // undo what we did
   //      }
   //   }
   //
   //   frameBuffer.end();
   //
   //   // libGDX renders this stuff upside down, so let's flip them right side up.
   //   // TODO: Does it really? Sneaking suspicion that it doesn't and we end up flipping things again later on.
   //   opencv_core.flip(rgba8888ColorImage.getBytedecoOpenCVMat(), rgba8888ColorImage.getBytedecoOpenCVMat(), BytedecoOpenCVTools.FLIP_Y);
   //   opencv_core.flip(normalizedDeviceCoordDepthImages.getBytedecoOpenCVMat(),
   //                    normalizedDeviceCoordDepthImages.getBytedecoOpenCVMat(),
   //                    BytedecoOpenCVTools.FLIP_Y);
   //
   //   opencv_core.randu(noiseImage.getBytedecoOpenCVMat(), noiseLow, noiseHigh);
   //
   //   parametersBuffer.getBytedecoFloatBufferPointer().put(0, camera.near);
   //   parametersBuffer.getBytedecoFloatBufferPointer().put(1, camera.far);
   //   parametersBuffer.getBytedecoFloatBufferPointer().put(2, principalOffsetXPixels);
   //   parametersBuffer.getBytedecoFloatBufferPointer().put(3, principalOffsetYPixels);
   //   parametersBuffer.getBytedecoFloatBufferPointer().put(4, focalLengthPixels);
   //   parametersBuffer.getBytedecoFloatBufferPointer().put(6, imageWidth);
   //   parametersBuffer.getBytedecoFloatBufferPointer().put(7, imageHeight);
   //   parametersBuffer.getBytedecoFloatBufferPointer().put(26, noiseAmplitudeAtMinRange);
   //   parametersBuffer.getBytedecoFloatBufferPointer().put(27, noiseAmplitudeAtMaxRange);
   //   parametersBuffer.getBytedecoFloatBufferPointer().put(28, simulateL515Noise);
   //   if (firstRender)
   //   {
   //      firstRender = false;
   //      normalizedDeviceCoordDepthImages.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
   //      noiseImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
   //      rgba8888ColorImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
   //      metersDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_WRITE_ONLY);
   //      pointCloudRenderingBuffer.createOpenCLBufferObject(openCLManager);
   //      parametersBuffer.createOpenCLBufferObject(openCLManager);
   //   }
   //   else
   //   {
   //      normalizedDeviceCoordDepthImages.writeOpenCLImage(openCLManager);
   //      noiseImage.writeOpenCLImage(openCLManager);
   //      rgba8888ColorImage.writeOpenCLImage(openCLManager);
   //      parametersBuffer.writeOpenCLBufferObject(openCLManager);
   //   }
   //   openCLManager.setKernelArgument(openCLKernel, 0, normalizedDeviceCoordDepthImages.getOpenCLImageObject());
   //   openCLManager.setKernelArgument(openCLKernel, 1, noiseImage.getOpenCLImageObject());
   //   openCLManager.setKernelArgument(openCLKernel, 2, rgba8888ColorImage.getOpenCLImageObject());
   //   openCLManager.setKernelArgument(openCLKernel, 3, metersDepthImage.getOpenCLImageObject());
   //   openCLManager.setKernelArgument(openCLKernel, 4, pointCloudRenderingBuffer.getOpenCLBufferObject());
   //   openCLManager.setKernelArgument(openCLKernel, 5, parametersBuffer.getOpenCLBufferObject());
   //   openCLManager.execute2D(openCLKernel, imageWidth, imageHeight);
   //   metersDepthImage.readOpenCLImage(openCLManager);
   //   pointCloudRenderingBuffer.readOpenCLBufferObject(openCLManager);
   //
   //}

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
      LibGDXTools.toEuclid(worldTransform, transformToWorldFrame);
   }

   public ByteBuffer getMetersDepthFloatBuffer()
   {
      return metersDepthImage.getBackingDirectByteBuffer();
   }

   public Mat getMetersDepthOpenCVMat()
   {
      return metersDepthImage.getBytedecoOpenCVMat();
   }

   public int getImageWidth()
   {
      return imageWidth;
   }

   public int getImageHeight()
   {
      return imageHeight;
   }

   public int getNumberOfPoints()
   {
      return numberOfPoints;
   }

   public Texture getFrameBufferColorTexture()
   {
      return frameBuffer.getColorTexture();
   }
}
