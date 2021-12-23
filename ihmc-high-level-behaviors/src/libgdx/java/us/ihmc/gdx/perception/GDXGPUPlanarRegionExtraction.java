package us.ihmc.gdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.Loader;
import org.bytedeco.opencl._cl_mem;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Size;
import us.ihmc.gdx.imgui.ImGuiVideoPanel;
import us.ihmc.perception.OpenCLManager;

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;

public class GDXGPUPlanarRegionExtraction
{
   private final ImFloat mergeDistanceThreshold = new ImFloat(0.016f);
   private final ImFloat mergeAngularThreshold = new ImFloat(0.82f);
   private final ImFloat filterDisparityThreshold = new ImFloat(2000);
   private final ImInt inputHeight = new ImInt(0);
   private final ImInt inputWidth = new ImInt(0);
   private final ImInt kernelSliderLevel = new ImInt(2);
   private final ImInt filterKernelSize = new ImInt(4);
   private final ImFloat depthFx = new ImFloat(0);
   private final ImFloat depthFy = new ImFloat(0);
   private final ImFloat depthCx = new ImFloat(0);
   private final ImFloat depthCy = new ImFloat(0);
   private final ImBoolean earlyGaussianBlur = new ImBoolean(true);
   private final ImInt gaussianSize = new ImInt(6);
   private final ImInt gaussianSigma = new ImInt(20);
   private final OpenCLManager openCLManager = new OpenCLManager();
   private int numberOfFloatParameters = 16;
   private _cl_mem parametersBufferObject;
   private long parametersBufferSizeInBytes;
   private FloatPointer parametersNativeCPUPointer;
   private Mat inputDepthImageMat;
//   private ByteBuffer depthImageBytePointer = new ByteBuffer();
//   private BytePointer depthImageBytePointer = new BytePointer();

   private final ImGuiVideoPanel blurredDepthPanel;
   private Pixmap blurredDepthPanelPixmap;
   private Texture blurredDepthPanelTexture;
   private int imageWidth;
   private int imageHeight;
   private float lowestValueSeen = -1.0f;
   private float highestValueSeen = -1.0f;

   public GDXGPUPlanarRegionExtraction()
   {
      blurredDepthPanel = new ImGuiVideoPanel("Blurred Depth", false);
   }

   public void create(int imageWidth, int imageHeight)
   {
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
      openCLManager.create();

      parametersBufferSizeInBytes = numberOfFloatParameters * Loader.sizeof(FloatPointer.class);
      parametersBufferObject = openCLManager.createBufferObject(parametersBufferSizeInBytes);
      parametersNativeCPUPointer = new FloatPointer(numberOfFloatParameters);

      inputDepthImageMat = new Mat(imageHeight, imageWidth, opencv_core.CV_16UC1);

      blurredDepthPanelPixmap = new Pixmap(imageWidth, imageHeight, Pixmap.Format.RGBA8888);
      blurredDepthPanelTexture = new Texture(new PixmapTextureData(blurredDepthPanelPixmap, null, false, false));

      blurredDepthPanel.setTexture(blurredDepthPanelTexture);
   }

   public void blurDepthAndRender(FloatBuffer depthFloatBuffer)
   {
      depthFloatBuffer.rewind();
      for (int y = 0; y < imageHeight; y++)
      {
         for (int x = 0; x < imageWidth; x++)
         {
            float eyeDepth = depthFloatBuffer.get();
            if (highestValueSeen < 0 || eyeDepth > highestValueSeen)
               highestValueSeen = eyeDepth;
            if (lowestValueSeen < 0 || eyeDepth < lowestValueSeen)
               lowestValueSeen = eyeDepth;

            float colorRange = highestValueSeen - lowestValueSeen;
            float grayscale = (eyeDepth - lowestValueSeen) / colorRange;
            int flippedY = imageHeight - y;

            blurredDepthPanelPixmap.drawPixel(x, flippedY, Color.rgba8888(grayscale, grayscale, grayscale, 1.0f));
         }
      }

      blurredDepthPanelTexture.draw(blurredDepthPanelPixmap, 0, 0);
   }

   private void generateRegionsFromDepth(FloatBuffer depthFloatBuffer)
   {
      // timestamp

      // put image into Mat?

//      depthImageBytePointer.putPointerValue(depthFloatBuffer);
//       inputDepthImageMat.


      generatePatchGraph();

      generateSegmentation();
   }

   private void generatePatchGraph()
   {
      uploadParametersToGPU();

      // gaussian blur
      Mat src = null;
      Mat dst = null;
      Size ksize = null;
      double sigmaX = 0.0;
      opencv_imgproc.GaussianBlur(src, dst, ksize, sigmaX);
   }

   private void generateSegmentation()
   {

   }

   private void uploadParametersToGPU()
   {
      int patchHeight = kernelSliderLevel.get();
      int patchWidth = kernelSliderLevel.get();
      int subHeight = inputHeight.get() / patchHeight;
      int subWidth = inputWidth.get() / patchWidth;
      int filterSubHeight = inputHeight.get() / filterKernelSize.get();
      int filterSubWidth = inputWidth.get() / filterKernelSize.get();

      FloatPointer parameters = new FloatPointer(numberOfFloatParameters);
      parameters.put(0, filterDisparityThreshold.get());
      parameters.put(1, mergeAngularThreshold.get());
      parameters.put(2, mergeDistanceThreshold.get());
      parameters.put(3, patchHeight);
      parameters.put(4, patchWidth);
      parameters.put(5, subHeight);
      parameters.put(6, subWidth);
      parameters.put(7, depthFx.get());
      parameters.put(8, depthFy.get());
      parameters.put(9, depthCx.get());
      parameters.put(10, depthCy.get());
      parameters.put(11, filterKernelSize.get());
      parameters.put(12, filterSubHeight);
      parameters.put(13, filterSubWidth);
      parameters.put(14, inputHeight.get());
      parameters.put(15, inputWidth.get());

      openCLManager.enqueueWriteBuffer(parametersBufferObject, parametersBufferSizeInBytes, parameters);
   }

   public ImGuiVideoPanel getBlurredDepthPanel()
   {
      return blurredDepthPanel;
   }
}
