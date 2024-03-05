package us.ihmc.perception.opticalFlow;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_cudaimgproc;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Size;
import org.bytedeco.opencv.opencv_core.Stream;
import org.bytedeco.opencv.opencv_cudaoptflow.NvidiaOpticalFlow_2_0;
import us.ihmc.perception.RawImage;

public class OpenCVDenseOpticalFlowProcessor
{
   private final NvidiaOpticalFlow_2_0 opticalFlow;
   private GpuMat firstImageGray = new GpuMat();
   private final GpuMat secondImageGray = new GpuMat();
   private final int gridSize;

   public OpenCVDenseOpticalFlowProcessor(RawImage initialImage)
   {
      Size imageSize = new Size(initialImage.getImageWidth(), initialImage.getImageHeight());
      opticalFlow = NvidiaOpticalFlow_2_0.create(imageSize,
                                                 NvidiaOpticalFlow_2_0.NV_OF_PERF_LEVEL_FAST,
                                                 NvidiaOpticalFlow_2_0.NV_OF_OUTPUT_VECTOR_GRID_SIZE_1,
                                                 NvidiaOpticalFlow_2_0.NV_OF_HINT_VECTOR_GRID_SIZE_1,
                                                 false,
                                                 false,
                                                 false,
                                                 0,
                                                 Stream.Null(),
                                                 Stream.Null());
      gridSize = opticalFlow.getGridSize();
      imageSize.close();
   }

   public void setFirstImage(RawImage firstImage)
   {
      firstImage.get();
      opencv_cudaimgproc.cvtColor(firstImage.getGpuImageMat(), firstImageGray, opencv_imgproc.COLOR_BGR2GRAY);
      firstImage.release();
   }

   public void setSecondImage(RawImage secondImage)
   {
      secondImage.get();
      opencv_cudaimgproc.cvtColor(secondImage.getGpuImageMat(), secondImageGray, opencv_imgproc.COLOR_BGR2GRAY);
      secondImage.release();
   }

   public void setNewImage(RawImage newImage)
   {
      newImage.get();
      firstImageGray.release();
      firstImageGray = secondImageGray.clone();
      opencv_cudaimgproc.cvtColor(newImage.getGpuImageMat(), secondImageGray, opencv_imgproc.COLOR_BGR2GRAY);
      newImage.release();
   }

   public GpuMat calculateFlow()
   {
      GpuMat flow = new GpuMat(firstImageGray.size().width() / gridSize, firstImageGray.size().height() / gridSize, opencv_core.CV_16SC2);
      GpuMat floatFlow = new GpuMat(flow.size().width(), flow.size().height(), opencv_core.CV_32FC2);

      opticalFlow.calc(firstImageGray, secondImageGray, flow);
      opticalFlow.convertToFloat(flow, floatFlow);
      flow.release();

      return floatFlow;
   }

   public void destroy()
   {
      opticalFlow.collectGarbage();
      opticalFlow.close();
      firstImageGray.release();
      secondImageGray.release();
   }
}
