package us.ihmc.perception.detections.yolo;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_dnn;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatExpr;
import org.bytedeco.opencv.opencv_core.MatVector;
import org.bytedeco.opencv.opencv_core.Rect;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.bytedeco.opencv.opencv_core.Size;
import org.bytedeco.opencv.opencv_core.StringVector;
import org.bytedeco.opencv.opencv_dnn.Net;
import org.yaml.snakeyaml.Yaml;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDANonMaximumSuppression;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.tools.Destroyable;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static org.bytedeco.cuda.global.cudart.*;

public class YOLOv8Model implements Destroyable
{
   private static final double SCALE_FACTOR = 1.0 / 255.0;
   private static final Size DETECTION_SIZE = new Size(1280, 736);

   private static final int FILTERED_FLOATS_PER_ROW = 38;
   private static final int FLOATS_PER_BOX = 5;

   private static final int BLOCK_SIZE_1D = 256;
   private static final int BLOCK_SIZE_2D = 16;

   // Meta data
   private final String modelName;
   private final List<String> detectionClassNames = new ArrayList<>();

   // OpenCV DNN stuff
   private final Net yoloNet;
   private final StringVector outputNames; // literally list of "output0", "output1", "output2"...

   // CUDA post-processing stuff
   private final CUstream_st cudaStream;
   private final CUDAProgram postProcessProgram;
   private final CUDAKernel filterKernel;
   private final CUDAKernel detectionMaskKernel;
   private final CUDANonMaximumSuppression nms;

   public YOLOv8Model(Path modelBaseDirectory)
   {
      // Ensure the passed in directory is a valid YOLO model directory
      if (!YOLOv8Tools.isValidYOLOModelDirectory(modelBaseDirectory))
         throw new IllegalArgumentException("Provided directory is not a YOLO model directory");

      // Get name & onnx file path
      modelName = modelBaseDirectory.getFileName().toString();

      try (InputStream inputStream = new FileInputStream(YOLOv8Tools.getClassNamesFile(modelBaseDirectory).toFile()))
      {
         // Parse class_names.yaml
         Yaml yaml = new Yaml();
         Map<String, List<String>> classNamesData = yaml.load(inputStream);
         List<String> names = classNamesData.get("names");
         detectionClassNames.addAll(names);

         // Read the YOLO net
         Path onnxFile = YOLOv8Tools.getONNXFile(modelBaseDirectory);
         yoloNet = opencv_dnn.readNetFromONNX(Files.readAllBytes(onnxFile));
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      // Use CUDA backends
      yoloNet.setPreferableBackend(opencv_dnn.DNN_BACKEND_CUDA);
      yoloNet.setPreferableTarget(opencv_dnn.DNN_TARGET_CUDA);

      outputNames = yoloNet.getUnconnectedOutLayersNames();

      // Initialize CUDA stuff
      cudaStream = CUDAStreamManager.getStream();
      try
      {
         URL postProcessProgramURL = YOLOv8Model.class.getResource("YOLOv8PostProcess.cu");
         postProcessProgram = new CUDAProgram(postProcessProgramURL, CUDATools.getUtilsFile());
         filterKernel = postProcessProgram.loadKernel("filterDetections");
         detectionMaskKernel = postProcessProgram.loadKernel("computeDetectionMask");
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }

      nms = new CUDANonMaximumSuppression();
   }

   @Override
   public void destroy()
   {
      yoloNet.close();
      outputNames.close();

      CUDAStreamManager.releaseStream(cudaStream);
      filterKernel.close();
      postProcessProgram.close();
      nms.close();
   }

   /**
    * Get the name of the model. The name of the model's directory is used.
    *
    * @return The name of the model.
    */
   public String getName()
   {
      return modelName;
   }

   /**
    * @return A list of object classes this model can detect.
    */
   public List<String> getDetectionClassNames()
   {
      return detectionClassNames;
   }

   /**
    * Run YOLO object detection and segmentation on the passed in {@code image}.
    *
    * @param image               Image to run YOLO on. If this image isn't in BGR format, it will be converted to BGR.
    *                            Pass in BGR images for optimal performance.
    * @param confidenceThreshold Minimum confidence detections must have to be considered valid [0.0, 1.0].
    * @param nmsThreshold        Non-maximum suppression threshold.
    * @param maskThreshold       Minimum value for a pixel to be part of the mask [0.0, 1.0].
    * @return List of {@link YOLOv8Detection}s found in the image.
    */
   public synchronized YOLOv8DetectionList run(RawImage image, float confidenceThreshold, float nmsThreshold, float maskThreshold)
   {
      YOLOv8DetectionList result = new YOLOv8DetectionList();

      // Ensure image is in BGR format
      RawImage bgrInputImage;
      if (image.getPixelFormat() == PixelFormat.BGR8)
      {
         bgrInputImage = image.get();
         if (bgrInputImage == null)
            return result;
      }
      else
      {
         Mat bgrMat = new Mat();
         image.getPixelFormat().convertToPixelFormat(image.getCpuImageMat(), bgrMat, PixelFormat.BGR8);
         bgrInputImage = image.replaceImage(bgrMat, PixelFormat.BGR8);
      }

      // Run the net
      MatVector outputBlobs = new MatVector();
      Mat blob = opencv_dnn.blobFromImage(bgrInputImage.getCpuImageMat(), SCALE_FACTOR, DETECTION_SIZE, new Scalar(), true, true, opencv_core.CV_32F);
      yoloNet.setInput(blob);
      yoloNet.forward(outputBlobs, outputNames);
      blob.close();

      // Get some useful stuff
      CameraIntrinsics maskIntrinsics = computeMaskIntrinsics(outputBlobs, bgrInputImage);
      int shiftWidth = (bgrInputImage.getWidth() - DETECTION_SIZE.width()) / 2;
      int shiftHeight = (bgrInputImage.getHeight() - DETECTION_SIZE.height()) / 2;

      /*
       * Output 0 contains data of all bounding boxes + mask weights detected by the model.
       * It is structured such that each column contains:
       *  - BBox center (X, Y) and dimensions (width & height)
       *  - Confidence values for each class (how confident the model is that this BBox bounds an object of a given class)
       *  - 32 mask weights associated with the 32 prototype masks (the prototype masks are contained in output 1)
       *
       * Typically, the model produces a ridiculous number of bounding boxes (~19K) for each run.
       * Most of those boxes are garbage with low confidences, and they must be discarded.
       * The good bounding boxes are collected, and put through non-maximum suppression to remove overlapping ones.
       * Finally, the bounding boxes and mask weights are combined with output 1 to get the object masks.
       *
       * OUTPUT 0:
       * BBox#      0   1   2   3   4   5   6   7   8       N
       *          +------------------------------------ . ----+
       * centerX  |   |   |   |   |   |   |   |   |   | . |   |
       *          |---+---+---+---+---+---+---+---+---+ . +---+
       * centerY  |   |   |   |   |   |   |   |   |   |   |   |
       *          |---+---+---+---+---+---+---+---+---+   +---+
       * width    |   |   |   |   |   |   |   |   |   |   |   |
       *          |---+---+---+---+---+---+---+---+---+   +---+
       * height   |   |   |   |   |   |   |   |   |   |   |   |
       *          |---+---+---+---+---+---+---+---+---+   +---+
       * cls0Conf |   |   |   |   |   |   |   |   |   |   |   |
       *          |---+---+---+---+---+---+---+---+---+   +---+
       * cls1Conf |   |   |   |   |   |   |   |   |   |   |   |
       *          |---+---+---+---+---+---+---+---+---+   +---+
       *          ...
       *          |---+---+---+---+---+---+---+---+---+   +---+
       * clsNConf |   |   |   |   |   |   |   |   |   |   |   |
       *          |---+---+---+---+---+---+---+---+---+   +---+
       * mskWht0  |   |   |   |   |   |   |   |   |   |   |   |
       *          |---+---+---+---+---+---+---+---+---+   +---+
       * mskWht1  |   |   |   |   |   |   |   |   |   |   |   |
       *          |---+---+---+---+---+---+---+---+---+   +---+
       * mskWht2  |   |   |   |   |   |   |   |   |   |   |   |
       *          |---+---+---+---+---+---+---+---+---+   +---+
       *          ...
       *          |---+---+---+---+---+---+---+---+---+ . +---+
       * mskWht32 |   |   |   |   |   |   |   |   |   | . |   |
       *          +------------------------------------ . ----+
       */
      try (Mat output0Blob = outputBlobs.get(0);
           Mat output1Blob = outputBlobs.get(1);

           FloatPointer unfilteredDetections = new FloatPointer();
           FloatPointer filteredDetections = new FloatPointer();
           IntPointer filteredDetectionCountPointer = new IntPointer();

           dim3 blockDims = new dim3();
           dim3 gridDims = new dim3();

           FloatPointer boxes = new FloatPointer();

           FloatPointer prototypeMasks = new FloatPointer())
      {
         int unfilteredFloatsPerDetection = output0Blob.size(1);
         int unfilteredDetectionCount = output0Blob.size(2);

         // Upload unfiltered results to GPU
         long totalUnfilteredFloats = (long) unfilteredFloatsPerDetection * unfilteredDetectionCount;
         CUDATools.mallocAsync(unfilteredDetections, totalUnfilteredFloats, cudaStream);
         CUDATools.checkCUDAError(cudaMemcpyAsync(unfilteredDetections,
                                                  output0Blob.data(),
                                                  unfilteredDetections.sizeof() * totalUnfilteredFloats,
                                                  cudaMemcpyDefault,
                                                  cudaStream));

         // Allocate memory for filtered detections
         long maxFilteredFloats = (long) FILTERED_FLOATS_PER_ROW * unfilteredDetectionCount;
         CUDATools.checkCUDAError(cudaMallocHost(filteredDetections, filteredDetections.sizeof() * maxFilteredFloats));
         CUDATools.checkCUDAError(cudaMallocHost(filteredDetectionCountPointer, filteredDetectionCountPointer.sizeof()));

         // Calculate kernel launch dimensions
         blockDims.x(BLOCK_SIZE_1D);
         gridDims.x((unfilteredDetectionCount + BLOCK_SIZE_1D - 1) / BLOCK_SIZE_1D);

         // Run the filter kernel
         filterKernel.withPointer(unfilteredDetections)
                     .withInt(detectionClassNames.size())
                     .withInt(unfilteredDetectionCount)
                     .withFloat(confidenceThreshold)
                     .withInt(shiftWidth)
                     .withInt(shiftHeight)
                     .withPointer(filteredDetections)
                     .withPointer(filteredDetectionCountPointer)
                     .run(cudaStream, gridDims, blockDims, 0);
         CUDATools.checkCUDAError(cudaStreamSynchronize(cudaStream));

         // Ensure that we have valid detections
         int filteredDetectionCount = filteredDetectionCountPointer.get();
         if (filteredDetectionCount == 0)
         {
            CUDATools.checkCUDAError(cudaFreeAsync(unfilteredDetections, cudaStream));
            CUDATools.checkCUDAError(cudaFreeHost(filteredDetections));
            CUDATools.checkCUDAError(cudaFreeHost(filteredDetectionCountPointer));
            return result;
         }

         // Copy boxes into separate memory to run NMS
         CUDATools.mallocAsync(boxes, (long) FLOATS_PER_BOX * filteredDetectionCount, cudaStream);
         cudaMemcpy2DAsync(boxes,
                           (long) boxes.sizeof() * FLOATS_PER_BOX,
                           filteredDetections,
                           (long) boxes.sizeof() * FILTERED_FLOATS_PER_ROW,
                           (long) boxes.sizeof() * FLOATS_PER_BOX,
                           filteredDetectionCount,
                           cudaMemcpyDefault,
                           cudaStream);
         CUDATools.checkCUDAError(cudaStreamSynchronize(cudaStream));

         // Run NMS on boxes
         IntPointer includedRows = new IntPointer(filteredDetectionCount);
         int remainingDetectionCount = nms.run(boxes, filteredDetectionCount, nmsThreshold, includedRows);

         // Ensure we still have detections
         if (remainingDetectionCount == 0)
         {
            CUDATools.checkCUDAError(cudaFreeAsync(unfilteredDetections, cudaStream));
            CUDATools.checkCUDAError(cudaFreeAsync(boxes, cudaStream));
            CUDATools.checkCUDAError(cudaFreeHost(filteredDetections));
            CUDATools.checkCUDAError(cudaFreeHost(filteredDetectionCountPointer));
            return result;
         }

         // Upload prototype masks to GPU
         long totalMaskFloats = (long) output1Blob.size(1) * output1Blob.size(2) * output1Blob.size(3);
         CUDATools.mallocAsync(prototypeMasks, totalMaskFloats, cudaStream);
         CUDATools.checkCUDAError(cudaMemcpyAsync(prototypeMasks,
                                                  output1Blob.data(),
                                                  prototypeMasks.sizeof() * totalMaskFloats,
                                                  cudaMemcpyDefault,
                                                  cudaStream));

         for (int i = 0; i < remainingDetectionCount; ++i)
         {
            int index = includedRows.get(i);
            FloatPointer row = filteredDetections.getPointer((long) FILTERED_FLOATS_PER_ROW * index);

            Rect boundingBox = new Rect(Math.round(row.get(0)), Math.round(row.get(1)), Math.round(row.get(2)), Math.round(row.get(3)));
            float confidence = row.get(4);
            int classID = (int) row.get(5);
            RawImage mask = computeDetectionMask(filteredDetections, prototypeMasks, index, maskThreshold, maskIntrinsics, bgrInputImage);

            YOLOv8Detection detection = new YOLOv8Detection(detectionClassNames.get(classID), confidence, boundingBox, mask);
            result.add(detection);

            boundingBox.close();
            mask.release();
         }

         includedRows.close();

         // Deallocate stuff
         CUDATools.checkCUDAError(cudaFreeAsync(unfilteredDetections, cudaStream));
         CUDATools.checkCUDAError(cudaFreeAsync(boxes, cudaStream));
         CUDATools.checkCUDAError(cudaFreeAsync(prototypeMasks, cudaStream));
         CUDATools.checkCUDAError(cudaFreeHost(filteredDetections));
         CUDATools.checkCUDAError(cudaFreeHost(filteredDetectionCountPointer));
      }

      outputBlobs.close();
      bgrInputImage.release();

      return result;
   }

   private CameraIntrinsics computeMaskIntrinsics(MatVector outputBlobs, RawImage bgrInputImage)
   {
      int maskHeight = outputBlobs.get(1).size(2);
      int maskWidth = outputBlobs.get(1).size(3);

      float xScaleFactor = (float) maskWidth / bgrInputImage.getWidth();
      float yScaleFactor = (float) maskHeight / bgrInputImage.getHeight();
      float maskFocalLengthX = xScaleFactor * bgrInputImage.getFocalLengthX();
      float maskFocalLengthY = yScaleFactor * bgrInputImage.getFocalLengthY();
      float maskPrincipalPointX = xScaleFactor * bgrInputImage.getPrincipalPointX();
      float maskPrincipalPointY = yScaleFactor * bgrInputImage.getPrincipalPointY();

      return new CameraIntrinsics(maskHeight, maskWidth, maskFocalLengthX, maskFocalLengthY, maskPrincipalPointX, maskPrincipalPointY);
   }

   private RawImage computeDetectionMask(FloatPointer filteredOutput,
                                         FloatPointer prototypeMasks,
                                         int detectionIndex,
                                         float maskThreshold,
                                         CameraIntrinsics maskIntrinsics,
                                         RawImage bgrInputImage)
   {
      GpuMat mask = new GpuMat(maskIntrinsics.getHeight(), maskIntrinsics.getWidth(), opencv_core.CV_8U);

      int gridX = (maskIntrinsics.getWidth() + BLOCK_SIZE_2D - 1) / BLOCK_SIZE_2D;
      int gridY = (maskIntrinsics.getHeight() + BLOCK_SIZE_2D - 1) / BLOCK_SIZE_2D;

      try (FloatPointer boundingBox = filteredOutput.getPointer((long) FILTERED_FLOATS_PER_ROW * detectionIndex);
           FloatPointer maskWeights = filteredOutput.getPointer((long) FILTERED_FLOATS_PER_ROW * detectionIndex + 6);
           dim3 blockSize = new dim3(BLOCK_SIZE_2D, BLOCK_SIZE_2D, 1);
           dim3 gridSize = new dim3(gridX, gridY, 1))
      {
         detectionMaskKernel.withPointer(prototypeMasks)
                            .withPointer(maskWeights)
                            .withPointer(boundingBox)
                            .withFloat(maskThreshold)
                            .withPointer(mask.cudaPtr())
                            .withLong(mask.step())
                            .withInt(maskIntrinsics.getWidth())
                            .withInt(maskIntrinsics.getHeight())
                            .run(cudaStream, gridSize, blockSize, 0);
         CUDATools.checkCUDAError(cudaStreamSynchronize(cudaStream));
      }

      return new RawImage(null,
                          mask,
                          PixelFormat.GRAY8,
                          new CameraIntrinsics(maskIntrinsics),
                          CameraModel.PINHOLE,
                          bgrInputImage.getPose(),
                          bgrInputImage.getAcquisitionTime(),
                          bgrInputImage.getSequenceNumber(),
                          bgrInputImage.getDepthDiscretization());
   }
}
