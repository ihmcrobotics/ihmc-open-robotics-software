package us.ihmc.perception.detections.yolo;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.BooleanPointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_dnn;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatVector;
import org.bytedeco.opencv.opencv_core.Rect;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.bytedeco.opencv.opencv_core.Size;
import org.bytedeco.opencv.opencv_core.StringVector;
import org.bytedeco.opencv.opencv_dnn.Net;
import org.yaml.snakeyaml.Yaml;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.RawImage;
import us.ihmc.sensors.CameraIntrinsics;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDANonMaximumSuppression;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.perception.imageMessage.PixelFormat;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.regex.Pattern;

import static org.bytedeco.cuda.global.cudart.*;

public class YOLOv8Model
{
   private static final double SCALE_FACTOR = 1.0 / 255.0;
   private static final Size DETECTION_SIZE = new Size(1280, 736);

   private static final int FILTERED_FLOATS_PER_ROW = 38;
   private static final int FLOATS_PER_BOX = 5;

   private static final int BLOCK_SIZE_1D = 256;
   private static final int BLOCK_SIZE_2D = 16;

   // Meta data
   private final String modelName;
   private final List<String> detectableObjects = new ArrayList<>();

   // OpenCV DNN stuff
   private final Net yoloNet;
   private final StringVector outputNames; // literally list of "output0", "output1"

   // CUDA post-processing stuff
   private final CUstream_st cudaStream;
   private final CUDAProgram postProcessProgram;
   private final CUDAKernel filterKernel;
   private final CUDAKernel detectionMaskKernel;
   private final CUDANonMaximumSuppression nms;

   // CPU memory
   private final MatVector outputBlobs = new MatVector();

   // CUDA memory
   private final FloatPointer unfilteredDetections = new FloatPointer();
   private final FloatPointer filteredDetections = new FloatPointer();
   private final IntPointer filteredDetectionCountPointer = new IntPointer();
   private final FloatPointer boxes = new FloatPointer();
   private final FloatPointer prototypeMasks = new FloatPointer();
   private boolean firstAllocation = true;

   // Parameters
   private final BooleanPointer ignoredObjectClasses = new BooleanPointer();
   private final FloatPointer confidenceThresholds = new FloatPointer();
   private final float[] maskThresholds;
   private float nmsThreshold;

   public YOLOv8Model(URL modelBaseDirectory)
   {
      // Get name & onnx file path
      String[] path = modelBaseDirectory.getPath().split(Pattern.quote(File.separator));
      modelName = path[path.length - 1];

      URL classNamesFileURL = YOLOv8Tools.getClassNamesFile(modelBaseDirectory);
      URL onnxFileURL = YOLOv8Tools.getONNXFile(modelBaseDirectory);

      try (InputStream classNamesFile = classNamesFileURL.openStream();
           InputStream onnxFile = onnxFileURL.openStream())
      {
         // Parse class_names.yaml
         Yaml yaml = new Yaml();
         Map<String, List<Object>> classNamesData = yaml.load(classNamesFile);
         List<Object> names = classNamesData.get("names");
         detectableObjects.addAll(names.stream().map(Object::toString).toList());

         // Read the YOLO net
         Objects.requireNonNull(onnxFile);
         yoloNet = opencv_dnn.readNetFromONNX(onnxFile.readAllBytes());
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
         postProcessProgram = new CUDAProgram(postProcessProgramURL, CUDATools.getUtilsFile(), CUDATools.getPerceptionUtilsFile());
         filterKernel = postProcessProgram.loadKernel("filterDetections");
         detectionMaskKernel = postProcessProgram.loadKernel("computeDetectionMask");
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }

      nms = new CUDANonMaximumSuppression();

      // Initialize parameters
      cudaMallocHost(ignoredObjectClasses, (long) ignoredObjectClasses.sizeof() * detectableObjects.size());
      cudaMallocHost(confidenceThresholds, (long) confidenceThresholds.sizeof() * detectableObjects.size());
      maskThresholds = new float[detectableObjects.size()];

      // Set parameters to default values
      boolean[] ignoredClasses = new boolean[detectableObjects.size()];
      Arrays.fill(ignoredClasses, false); // Don't ignore any by default
      setIgnoredClasses(ignoredClasses);
      setConfidenceThresholds(0.7f);
      setMaskThresholds(0.0f);
      setNMSThreshold(0.2f);
   }

   public synchronized void destroy()
   {
      yoloNet.close();
      outputNames.close();
      outputBlobs.close();

      freeCUDAMemory();

      unfilteredDetections.close();
      filteredDetections.close();
      filteredDetectionCountPointer.close();
      boxes.close();
      prototypeMasks.close();

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
   public List<String> getDetectableObjects()
   {
      return detectableObjects;
   }

   public int getDetectableObjectCount()
   {
      return detectableObjects.size();
   }

   /**
    * Set whether to ignore detections of a certain object class
    *
    * @param objectClass Name of the object class
    * @param ignore      If true, detections of the object class will be ignored.
    */
   public void ignore(String objectClass, boolean ignore)
   {
      ignoredObjectClasses.put(detectableObjects.indexOf(objectClass), ignore);
   }

   /**
    * Set the object class detections to ignore.
    *
    * @param ignoredClasses An array of booleans, each determining whether the corresponding (by index) object class will be ignored.
    */
   public void setIgnoredClasses(boolean[] ignoredClasses)
   {
      ignoredObjectClasses.put(ignoredClasses);
   }

   /**
    * Set the mask threshold for all object classes. A higher value will generally shrink the mask.
    *
    * @param maskThreshold Minimum value for a pixel to be part of the mask.
    */
   public void setMaskThresholds(float maskThreshold)
   {
      Arrays.fill(maskThresholds, maskThreshold);
   }

   /**
    * Set the mask threshold for each object class. A higher value will generally shrink the mask.
    *
    * @param objectClassMaskThresholds Minimum values for a pixel to be part of the mask.
    */
   public void setMaskThresholds(float[] objectClassMaskThresholds)
   {
      System.arraycopy(objectClassMaskThresholds, 0, maskThresholds, 0, maskThresholds.length);
   }

   /**
    * Set the confidence thresholds for all object classes
    *
    * @param confidenceThreshold Minimum confidence value detections must have to be considered valid [0.0, 1.0].
    */
   public void setConfidenceThresholds(float confidenceThreshold)
   {
      float[] confidenceArray = new float[detectableObjects.size()];
      Arrays.fill(confidenceArray, confidenceThreshold);
      setConfidenceThresholds(confidenceArray);
   }

   /**
    * Set the confidence threshold for each object class
    *
    * @param objectClassConfidenceThresholds Array of minimum confidence values detections must have to be considered valid [0.0, 1.0].
    */
   public void setConfidenceThresholds(float[] objectClassConfidenceThresholds)
   {
      confidenceThresholds.put(objectClassConfidenceThresholds);
   }

   /**
    * Set the non-maximum suppression threshold for determining whether boxes overlap.
    * The smaller the value, the less overlap is required for a box to be removed.
    *
    * @param nmsThreshold Non-maximum suppression threshold [0.0, 1.0]
    */
   public void setNMSThreshold(float nmsThreshold)
   {
      this.nmsThreshold = nmsThreshold;
   }

   /**
    * Run YOLO object detection and segmentation on the passed in {@code image}.
    *
    * @param image Image to run YOLO on. If this image isn't in BGR format, it will be converted to BGR.
    *              Pass in BGR images for optimal performance.
    * @return List of {@link YOLOv8Detection}s found in the image.
    */
   public synchronized YOLOv8DetectionList run(RawImage image)
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
      Mat blob = opencv_dnn.blobFromImage(bgrInputImage.getCpuImageMat(), SCALE_FACTOR, DETECTION_SIZE, new Scalar(), true, true, opencv_core.CV_32F);
      yoloNet.setInput(blob);
      yoloNet.forward(outputBlobs, outputNames);
      blob.close();

      /*
       * Output 0 contains data of all bounding boxes + mask weights detected by the model.
       * It is structured such that each column contains:
       *  - BBox center (X, Y) and dimensions (width & height)
       *  - Confidence values for each class (how confident the model is that this BBox bounds an object of a given class)
       *  - 32 mask weights associated with the 32 prototype masks (the prototype masks are contained in output 1)
       *
       * Typically, the model produces a ridiculous number of bounding boxes (~19K) for each run.
       * Most of those boxes are garbage with low confidences, and they must be filtered out.
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

           dim3 blockDims = new dim3();
           dim3 gridDims = new dim3())
      {
         updateCUDAMemoryAllocation(output0Blob, output1Blob);

         int unfilteredFloatsPerDetection = output0Blob.size(1);
         int unfilteredDetectionCount = output0Blob.size(2);

         // Upload unfiltered results to GPU
         long totalUnfilteredFloats = (long) unfilteredFloatsPerDetection * unfilteredDetectionCount;
         CUDATools.checkCUDAError(cudaMemcpyAsync(unfilteredDetections,
                                                  output0Blob.data(),
                                                  unfilteredDetections.sizeof() * totalUnfilteredFloats,
                                                  cudaMemcpyDefault,
                                                  cudaStream));

         // Ensure filtered detection count begins at 0
         filteredDetectionCountPointer.put(0);

         // Calculate kernel launch dimensions
         blockDims.x(BLOCK_SIZE_1D);
         gridDims.x((unfilteredDetectionCount + BLOCK_SIZE_1D - 1) / BLOCK_SIZE_1D);

         // Run the filter kernel
         filterKernel.withPointer(unfilteredDetections)
                     .withInt(detectableObjects.size())
                     .withInt(unfilteredDetectionCount)
                     .withPointer(confidenceThresholds)
                     .withPointer(ignoredObjectClasses)
                     .withPointer(filteredDetections)
                     .withPointer(filteredDetectionCountPointer)
                     .run(cudaStream, gridDims, blockDims, 0);
         CUDATools.checkCUDAError(cudaStreamSynchronize(cudaStream));

         // Ensure that we have valid detections
         int filteredDetectionCount = filteredDetectionCountPointer.get();
         if (filteredDetectionCount == 0)
            return result;

         // Copy boxes into separate memory to run NMS
         cudaMemcpy2DAsync(boxes,
                           (long) boxes.sizeof() * FLOATS_PER_BOX,
                           filteredDetections,
                           (long) boxes.sizeof() * FILTERED_FLOATS_PER_ROW,
                           (long) boxes.sizeof() * FLOATS_PER_BOX,
                           filteredDetectionCount,
                           cudaMemcpyDefault,
                           cudaStream);

         // Run NMS on boxes
         IntPointer includedRows = new IntPointer(filteredDetectionCount);
         int remainingDetectionCount = nms.runAsync(boxes, filteredDetectionCount, nmsThreshold, includedRows, cudaStream);

         // Ensure we still have detections
         if (remainingDetectionCount == 0)
            return result;

         // Upload prototype masks to GPU
         long totalMaskFloats = (long) output1Blob.size(1) * output1Blob.size(2) * output1Blob.size(3);
         CUDATools.checkCUDAError(cudaMemcpyAsync(prototypeMasks,
                                                  output1Blob.data(),
                                                  prototypeMasks.sizeof() * totalMaskFloats,
                                                  cudaMemcpyDefault,
                                                  cudaStream));

         // Get some useful stuff
         CameraIntrinsics maskIntrinsics = computeMaskIntrinsics(bgrInputImage);
         float widthScale = (float) bgrInputImage.getWidth() / DETECTION_SIZE.width();
         float heightScale = (float) bgrInputImage.getHeight() / DETECTION_SIZE.height();
         float scaleFactor = Math.min(widthScale, heightScale);
         float scaledDetectionWidth = DETECTION_SIZE.width() * scaleFactor;
         float scaledDetectionHeight = DETECTION_SIZE.height() * scaleFactor;
         float offsetX = 0.5f * (bgrInputImage.getWidth() - scaledDetectionWidth);
         float offsetY = 0.5f * (bgrInputImage.getHeight() - scaledDetectionHeight);

         // Create the list of YOLOv8Detections
         for (int i = 0; i < remainingDetectionCount; ++i)
         {
            int index = includedRows.get(i);
            FloatPointer row = filteredDetections.getPointer((long) FILTERED_FLOATS_PER_ROW * index);

            Rect boundingBox = new Rect(Math.round(scaleFactor * row.get(0) + offsetX),
                                        Math.round(scaleFactor * row.get(1) + offsetY),
                                        Math.round(scaleFactor * row.get(2)),
                                        Math.round(scaleFactor * row.get(3)));
            float confidence = row.get(4);
            int classID = (int) row.get(5);
            RawImage mask = computeDetectionMask(filteredDetections, prototypeMasks, index, maskThresholds[classID], maskIntrinsics, bgrInputImage);

            YOLOv8Detection detection = new YOLOv8Detection(detectableObjects.get(classID), classID, confidence, boundingBox, mask);
            result.add(detection);

            boundingBox.close();
            mask.release();
         }

         includedRows.close();
      }

      bgrInputImage.release();

      return result;
   }

   private CameraIntrinsics computeMaskIntrinsics(RawImage bgrInputImage)
   {
      int maskHeight = outputBlobs.get(1).size(2);
      int maskWidth = outputBlobs.get(1).size(3);

      float scaleFactor = Math.max((float) maskWidth / bgrInputImage.getWidth(), (float) maskHeight / bgrInputImage.getHeight());

      float offsetX = 0.5f * (maskWidth - scaleFactor * bgrInputImage.getWidth());
      float offsetY = 0.5f * (maskHeight - scaleFactor * bgrInputImage.getHeight());

      float maskFocalLengthX = scaleFactor * bgrInputImage.getFocalLengthX();
      float maskFocalLengthY = scaleFactor * bgrInputImage.getFocalLengthY();
      float maskPrincipalPointX = scaleFactor * bgrInputImage.getPrincipalPointX() + offsetX;
      float maskPrincipalPointY = scaleFactor * bgrInputImage.getPrincipalPointY() + offsetY;

      return new CameraIntrinsics(maskHeight, maskWidth, maskFocalLengthX, maskFocalLengthY, maskPrincipalPointX, maskPrincipalPointY);
   }

   private void updateCUDAMemoryAllocation(Mat output0Blob, Mat output1Blob)
   {
      // Ensure enough memory allocated for unfiltered detections
      int unfilteredFloatsPerDetection = output0Blob.size(1);
      int unfilteredDetectionCount = output0Blob.size(2);
      long totalUnfilteredFloats = (long) unfilteredFloatsPerDetection * unfilteredDetectionCount;
      if (firstAllocation || unfilteredDetections.limit() < totalUnfilteredFloats)
      {
         if (!firstAllocation)
            CUDATools.checkCUDAError(cudaFreeAsync(unfilteredDetections, cudaStream));
         CUDATools.mallocAsync(unfilteredDetections, totalUnfilteredFloats, cudaStream);
         unfilteredDetections.limit(totalUnfilteredFloats);
      }

      // Ensure enough memory allocated for filtered detections
      long maxFilteredFloats = (long) FILTERED_FLOATS_PER_ROW * unfilteredDetectionCount;
      if (firstAllocation || filteredDetections.limit() < maxFilteredFloats)
      {
         if (!firstAllocation)
            CUDATools.checkCUDAError(cudaFreeHost(filteredDetections));
         CUDATools.checkCUDAError(cudaMallocHost(filteredDetections, filteredDetections.sizeof() * maxFilteredFloats));
         filteredDetections.limit(maxFilteredFloats);
      }
      if (firstAllocation)
         CUDATools.checkCUDAError(cudaMallocHost(filteredDetectionCountPointer, filteredDetectionCountPointer.sizeof()));

      // Ensure enough memory allocated for boxes
      long maxBoxFloats = (long) FLOATS_PER_BOX * unfilteredDetectionCount;
      if (firstAllocation || boxes.limit() < maxBoxFloats)
      {
         if (!firstAllocation)
            CUDATools.checkCUDAError(cudaFreeAsync(boxes, cudaStream));
         CUDATools.mallocAsync(boxes, maxBoxFloats, cudaStream);
         boxes.limit(maxBoxFloats);
      }

      // Ensure enough memory allocated for prototype masks
      long totalMaskFloats = (long) output1Blob.size(1) * output1Blob.size(2) * output1Blob.size(3);
      if (firstAllocation || prototypeMasks.limit() < totalMaskFloats)
      {
         if (!firstAllocation)
            CUDATools.checkCUDAError(cudaFreeAsync(prototypeMasks, cudaStream));
         CUDATools.mallocAsync(prototypeMasks, totalMaskFloats, cudaStream);
         prototypeMasks.limit(totalMaskFloats);
      }

      firstAllocation = false;
   }

   private void freeCUDAMemory()
   {
      if (firstAllocation)
         return;

      CUDATools.checkCUDAError(cudaFreeAsync(unfilteredDetections, cudaStream));
      CUDATools.checkCUDAError(cudaFreeAsync(boxes, cudaStream));
      CUDATools.checkCUDAError(cudaFreeAsync(prototypeMasks, cudaStream));
      CUDATools.checkCUDAError(cudaFreeHost(filteredDetections));
      CUDATools.checkCUDAError(cudaFreeHost(filteredDetectionCountPointer));
      CUDATools.checkCUDAError(cudaFreeHost(ignoredObjectClasses));
      CUDATools.checkCUDAError(cudaFreeHost(confidenceThresholds));
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
                          bgrInputImage.getTransformToWorld(),
                          bgrInputImage.getAcquisitionTime(),
                          bgrInputImage.getSequenceNumber(),
                          bgrInputImage.getDepthDiscretization());
   }
}
