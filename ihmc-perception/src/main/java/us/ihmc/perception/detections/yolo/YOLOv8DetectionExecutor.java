package us.ihmc.perception.detections.yolo;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Size;
import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.YOLOv8ExecutorParameters;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.cuda.CUDADepthImageSegmenter;
import us.ihmc.perception.cuda.CUDAPointCloudExtractor;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;

import java.net.URL;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class YOLOv8DetectionExecutor
{
   private final ROS2Node ros2Node = new ROS2NodeBuilder().build("yolo_detection_manager");

   private final CUDAPointCloudExtractor extractor;
   private final CUDADepthImageSegmenter segmenter;

   private final Map<String, YOLOv8Model> availableModels = new LinkedHashMap<>();
   private List<YOLOv8InstantDetection> annotatedImageDetections = new ArrayList<>();
   private Iterator<YOLOv8Model> modelIterator;

   private final SyncedYOLOv8ExecutorParameters parameters;
   private boolean requestingFullData;
   private final ROS2Publisher<YOLOv8ExecutorParameters> parametersPublisher;
   private final YOLOv8ExecutorParameters parametersMessage;
   private final RepeatingTaskThread updateThread;

   private final List<Consumer<List<InstantDetection>>> detectionConsumerCallbacks = new ArrayList<>();

   private final ROS2Publisher<ImageMessage> annotatedImagePublisher = ros2Node.createPublisher(PerceptionAPI.YOLO_ANNOTATED_IMAGE);
   private final BooleanSupplier annotatedImageDemanded;
   private final RepeatingTaskThread annotatedImagePublishedThread;
   private final TypedNotification<RawImage> newestColorImage = new TypedNotification<>();

   private final BoTSortTracker botSortTracker = new BoTSortTracker();

   private final BlockingQueue<Runnable> taskQueue;
   private final RepeatingTaskThread taskExecutorThread = new RepeatingTaskThread("YOLOExecutor", this::executeTasks, DefaultExceptionHandler.RUNTIME_EXCEPTION);

   public YOLOv8DetectionExecutor(ROS2PeerClockOffsetEstimator peerClockEstimator, BooleanSupplier annotatedImageDemanded)
   {
      this.annotatedImageDemanded = annotatedImageDemanded;

      CRDTInfo crdtInfo = new CRDTInfo(ROS2ActorDesignation.ROBOT, peerClockEstimator);

      try
      {
         extractor = new CUDAPointCloudExtractor();
         segmenter = new CUDADepthImageSegmenter();
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }

      // Read available YOLO models
      for (URL yoloModelDirectory : YOLOv8Tools.getYOLOModelDirectories())
      {
         YOLOv8Model model = new YOLOv8Model(yoloModelDirectory);

         LogTools.info("Loaded YOLOv8 model: " + model.getName());
         LogTools.info("\t\t\tClasses: " + model.getDetectableObjectCount());

         availableModels.put(model.getName(), model);
      }

      if (availableModels.isEmpty())
         LogTools.error("No YOLO models found. YOLO will not run.");

      // Create YOLO parameters
      parameters = new SyncedYOLOv8ExecutorParameters(crdtInfo);
      parameters.setAvailableModels(availableModels.values());
      parameters.requestSendFullData();
      requestingFullData = true;

      // Subscribe to YOLO parameters messages
      ros2Node.createSubscription2(PerceptionAPI.YOLO_PARAMETERS, message ->
      {
         parameters.fromMessage(message);
         parameters.confirmReceivedFullData();
         requestingFullData = false;
      });

      parametersPublisher = ros2Node.createPublisher(PerceptionAPI.YOLO_PARAMETERS);
      parametersMessage = new YOLOv8ExecutorParameters();
      updateThread = new RepeatingTaskThread(getClass().getSimpleName() + "Updater", this::update);
      updateThread.setFrequencyLimit(10.0);
      updateThread.setDaemon(true);
      updateThread.startRepeating();

      taskQueue = new ArrayBlockingQueue<>(4);
      taskExecutorThread.setDaemon(true);
      taskExecutorThread.startRepeating();

      annotatedImagePublishedThread = new RepeatingTaskThread("YOLOAnnotatedImagePublisher", this::annotateAndPublishImage, DefaultExceptionHandler.RUNTIME_EXCEPTION);
      annotatedImagePublishedThread.setDaemon(true);
      annotatedImagePublishedThread.startRepeating();
   }

   public void addDetectionConsumerCallback(Consumer<List<InstantDetection>> callback)
   {
      detectionConsumerCallbacks.add(callback);
   }

   public List<String> getAvailableModelNames()
   {
      return new ArrayList<>(availableModels.keySet());
   }

   public void enableModel(String modelName)
   {
      parameters.getModelsToRun().add(modelName);
   }

   public void enableAllModels()
   {
      parameters.getModelsToRun().addAll(availableModels.keySet());
   }

   public void disableModel(String modelName)
   {
      parameters.getModelsToRun().remove(modelName);
   }

   public void disableAllModels()
   {
      parameters.getModelsToRun().clear();
   }

   public void runNextEnabledModel(RawImage colorImage, RawImage depthImage)
   {
      for (int i = 0; i < availableModels.size(); ++i)
      {
         if (modelIterator == null || !modelIterator.hasNext())
            modelIterator = availableModels.values().iterator();

         YOLOv8Model model = modelIterator.next();

         if (parameters.getModelsToRun().getValue().contains(model.getName()))
         {
            runYOLODetection(model, colorImage, depthImage);
            return;
         }

         annotatedImageDetections.remove(model);
      }

      while (modelIterator.hasNext())
      {
         YOLOv8Model model = modelIterator.next();

         if (parameters.getModelsToRun().getValue().contains(model.getName()))
         {
            runYOLODetection(model, colorImage, depthImage);
            return;
         }

         annotatedImageDetections.remove(model);
      }
   }

   /**
    * Non-blocking call to run YOLO on the provided images
    * @param colorImage BGR color image, used for YOLO detection
    * @param depthImage 16UC1 depth image, used to get points of detected objects
    */
   private void runYOLODetection(YOLOv8Model yoloModel, RawImage colorImage, RawImage depthImage)
   {
      if (taskQueue.remainingCapacity() > 0)
      {
         // Acquire the images
         if (colorImage.get() == null || depthImage.get() == null)
            return;

         taskQueue.add(() ->
         {
            // Run YOLO to get results
            GpuMat bgrMat = new GpuMat();
            colorImage.getPixelFormat().convertToPixelFormat(colorImage.getGpuImageMat(), bgrMat, PixelFormat.BGR8);
            RawImage bgrImage = colorImage.replaceImage(bgrMat, PixelFormat.BGR8);
            YOLOv8DetectionList yoloResults = yoloModel.run(bgrImage);

            if (newestColorImage.poll())
               newestColorImage.read().release();
            newestColorImage.set(bgrImage.get());

            SyncedYOLOv8ModelParameters modelParameters = parameters.getModelParameters().get(yoloModel.getName());

            // Create list of instant detections from results
            List<InstantDetection> yoloInstantDetections = new ArrayList<>();
            List<YOLOv8InstantDetection> trackableDetections = new ArrayList<>();
            List<YOLOv8InstantDetection> newAnnotatedImageDetections = new ArrayList<>();
            for (YOLOv8Detection detection : yoloResults)
            {
               RawImage objectMask = detection.mask();

               // Erode mask to get better segmentation
               int erosionKernelRadius = modelParameters.getErosionKernelRadii().getValueReadOnly(detection.objectClassID());
               Mat erodedMask = new Mat(objectMask.getHeight(), objectMask.getWidth(), objectMask.getOpenCVType());
               opencv_imgproc.erode(objectMask.getCpuImageMat(),
                                    erodedMask,
                                    opencv_imgproc.getStructuringElement(opencv_imgproc.CV_SHAPE_RECT,
                                                                         new Size(2 * erosionKernelRadius + 1, 2 * erosionKernelRadius + 1),
                                                                         new Point(erosionKernelRadius, erosionKernelRadius)));
               RawImage erodedObjectMask = objectMask.replaceImage(erodedMask);
               objectMask.release();

               // Get the segmented depth image
               RawImage segmentedDepth = segmenter.removeBackground(depthImage, erodedObjectMask);
               if (segmentedDepth == null)
               {
                  erodedObjectMask.release();
                  continue;
               }
               // Get the point cloud
               List<Point3D32> pointCloud = extractor.extractPointCloud(segmentedDepth);
               // Filter out outliers from the point cloud
               float outlierThreshold = modelParameters.getOutlierThresholds().getValueReadOnly(detection.objectClassID());
               List<Point3D32> filteredPoints = YOLOv8Tools.filterOutliers(pointCloud, outlierThreshold, 128);

               if (!filteredPoints.isEmpty())
               {
                  Point3D32 centroid = YOLOv8Tools.computeCentroidOfPointCloud(filteredPoints, 128);

//                  annotatedImageDetections.add(new YOLOv8InstantDetection(detection.objectClass(),
//                                                                          detection.confidence(),
//                                                                          new Pose3D(centroid, new RotationMatrix()),
//                                                                          erodedObjectMask.getAcquisitionTime(),
//                                                                          bgrImage,
//                                                                          erodedObjectMask,
//                                                                          depthImage,
//                                                                          detection.boundingBox(),
//                                                                          filteredPoints));
                  YOLOv8InstantDetection det = new YOLOv8InstantDetection(detection.objectClass(),
                                                                       detection.confidence(),
                                                                       new Pose3D(centroid, new RotationMatrix()),
                                                                       erodedObjectMask.getAcquisitionTime(),
                                                                       bgrImage,
                                                                       erodedObjectMask,
                                                                       depthImage,
                                                                       detection.boundingBox(),
                                                                       filteredPoints);
                  trackableDetections.add(det);
                  yoloInstantDetections.add(det);
                  newAnnotatedImageDetections.add(det);
               }
               erodedObjectMask.release();
               segmentedDepth.release();
            }

            // Track IDs
            botSortTracker.update(trackableDetections);

            // Callbacks exactly once
            detectionConsumerCallbacks.forEach(cb -> cb.accept(yoloInstantDetections));

            // Update annotated detections list (destroy previous)
            synchronized (annotatedImagePublishedThread)
            {
               List<YOLOv8InstantDetection> previous = this.annotatedImageDetections;
               this.annotatedImageDetections = newAnnotatedImageDetections;
               for (YOLOv8InstantDetection old : previous)
                  old.destroy();
            }

            // Process callbacks
//            if (!yoloInstantDetections.isEmpty())
//            {
//               detectionConsumerCallbacks.forEach(callback -> callback.accept(yoloInstantDetections));
//
//               synchronized (annotatedImagePublishedThread)
//               {
//                  List<YOLOv8InstantDetection> previousAnnotatedImageDetections = this.annotatedImageDetections;
//                  this.annotatedImageDetections = annotatedImageDetections;
//                  for (YOLOv8InstantDetection previousAnnotatedImageDetection : previousAnnotatedImageDetections)
//                     previousAnnotatedImageDetection.destroy();
//               }
//            }

            yoloResults.destroy();
            bgrImage.release();
            colorImage.release();
            depthImage.release();
         });
      }
   }

   public void destroy()
   {
      System.out.println("Destroying " + getClass().getSimpleName());
      updateThread.blockingKill();

      ros2Node.destroy();

      taskExecutorThread.kill();
      taskExecutorThread.interrupt();

      annotatedImagePublishedThread.kill();
      newestColorImage.set(null);

      for (YOLOv8Model yoloModel : availableModels.values())
         yoloModel.destroy();

      extractor.close();
      segmenter.close();

      System.out.println("Destroyed " + getClass().getSimpleName());
   }

   private void executeTasks()
   {
      try
      {
         taskQueue.take().run();
      }
      catch (InterruptedException ignored) {}
   }

   private void annotateAndPublishImage()
   {
      RawImage colorImage = newestColorImage.blockingPoll();
      if (colorImage == null)
         return;

      if (!annotatedImageDemanded.getAsBoolean())
      {
         colorImage.release();
         return;
      }

      Mat resultMat = new Mat();
      synchronized (annotatedImagePublishedThread)
      {
         YOLOv8Tools.annotateImage(colorImage.getCpuImageMat(), resultMat, annotatedImageDetections);
      }

      BytePointer annotatedImagePointer = new BytePointer();
      opencv_imgcodecs.imencode(".jpg", resultMat, annotatedImagePointer); // for some reason using CUDAImageEncoder broke YOLO's CUDNN

      ImageMessage imageMessage = new ImageMessage();
      PerceptionMessageTools.packImageMessage(colorImage, annotatedImagePointer, CompressionType.JPEG, imageMessage);
      annotatedImagePublisher.publish(imageMessage);

      resultMat.close();
      colorImage.release();
   }

   private void update()
   {
      parameters.checkModified();
      parameters.getModelParameters().forEach((modelName, modelParameters) ->
      {
         modelParameters.checkModified();
         if (modelParameters.isModified())
            modelParameters.applyToModel(availableModels.get(modelName));
      });

      if (requestingFullData || parameters.pollNeedSendFullData() || parameters.getModelParameters().values().stream().anyMatch(LatestTimestampModifiable::pollNeedSendFullData))
      {
         parameters.toMessage(parametersMessage);
         parametersPublisher.publish(parametersMessage);
      }
   }

   public Map<String, SyncedYOLOv8ModelParameters> getModelParameters()
   {
      return parameters.getModelParameters();
   }
}
