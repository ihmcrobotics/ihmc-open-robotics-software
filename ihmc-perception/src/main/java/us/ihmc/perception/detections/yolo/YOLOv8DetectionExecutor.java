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
import us.ihmc.perception.detections.botsortTracker.BoTSORTTrack;
import us.ihmc.perception.detections.botsortTracker.BoTSORTTracker;
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
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class YOLOv8DetectionExecutor
{
   private final ROS2Node ros2Node = new ROS2NodeBuilder().build("yolo_detection_manager");

   private final CUDAPointCloudExtractor extractor;
   private final CUDADepthImageSegmenter segmenter;

   private final Map<String, YOLOv8Model> availableModels = new LinkedHashMap<>();
   private final Map<YOLOv8Model, YOLOv8DetectionList> yoloDetectionResults = new ConcurrentHashMap<>();
   private Iterator<YOLOv8Model> modelIterator;

   private final SyncedYOLOv8ExecutorParameters parameters;
   private boolean requestingFullData;
   private final ROS2Publisher<YOLOv8ExecutorParameters> parametersPublisher;
   private final YOLOv8ExecutorParameters parametersMessage;
   private final RepeatingTaskThread updateThread;

   private final List<Consumer<List<InstantDetection>>> detectionConsumerCallbacks = new ArrayList<>();

   private final ROS2Publisher<ImageMessage> annotatedImagePublisher = ros2Node.createPublisher(PerceptionAPI.YOLO_ANNOTATED_IMAGE);
   private final ROS2Publisher<ImageMessage> trackedAnnotatedImagePublisher = ros2Node.createPublisher(PerceptionAPI.YOLO_TRACKED_ANNOTATED_IMAGE);
   private final BooleanSupplier annotatedImageDemanded;
   private final RepeatingTaskThread annotatedImagePublishedThread;
   private final TypedNotification<RawImage> newestColorImage = new TypedNotification<>();

   private final BlockingQueue<Runnable> taskQueue;
   private final RepeatingTaskThread taskExecutorThread = new RepeatingTaskThread("YOLOExecutor", this::executeTasks, DefaultExceptionHandler.RUNTIME_EXCEPTION);
   private final BoTSORTTracker botSort = new BoTSORTTracker(
         /*historySize*/        10,
         /*startThresh*/        0.50f,
         /*stopThresh*/         0.30f,
         /*minFrameCount*/      10,
         /*maxMissedFrames*/    3,
         /*minAreaRatio*/       0.001f,
         /*maxAreaRatio*/       0.10f,
         /*texMin*/             0.0f,
         /*texMax*/             1.0f,
         /*borderSafeDistance*/ 50f,
         /*wProb*/              1.0f,
         /*wTemporal*/          1.0f,
         /*wTexture*/           1.0f,
         /*wBorder*/            1.0f,
         /*wSize*/              1.0f
   );
   // store latest published tracks so the annotator thread can overlay IDs
   private final Map<YOLOv8Model, List<BoTSORTTrack>> latestPublishedTracks = new ConcurrentHashMap<>();
   public YOLOv8DetectionExecutor(CRDTInfo crdtInfo, BooleanSupplier annotatedImageDemanded)
   {
      this.annotatedImageDemanded = annotatedImageDemanded;

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
      if (modelIterator == null || !modelIterator.hasNext())
         modelIterator = availableModels.values().iterator();

      while (modelIterator.hasNext())
      {
         YOLOv8Model model = modelIterator.next();

         if (parameters.getModelsToRun().getValue().contains(model.getName()))
         {
            runYOLODetection(model, colorImage, depthImage);
            return;
         }

         yoloDetectionResults.remove(model);
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

            // Convert raw YOLO results to 2D lists for the tracker (fast path: 2D only)
            final int frameW = bgrImage.getWidth();
            final int frameH = bgrImage.getHeight();
            final List<float[]> tBBoxes = new ArrayList<>(yoloResults.size());
            final List<Float>   tProbs  = new ArrayList<>(yoloResults.size());
            final List<String>  tNames  = new ArrayList<>(yoloResults.size());

            for (YOLOv8Detection d : yoloResults) {
               // Use the rectangle for simplicity
               var r = d.boundingBoxRect();
               float x1 = r.x();
               float y1 = r.y();
               float x2 = r.x() + r.width();
               float y2 = r.y() + r.height();
               r.close();
               // clamp (defensive)
               x1 = Math.max(0, Math.min(x1, frameW - 1));
               y1 = Math.max(0, Math.min(y1, frameH - 1));
               x2 = Math.max(0, Math.min(x2, frameW - 1));
               y2 = Math.max(0, Math.min(y2, frameH - 1));
               if (x2 <= x1 || y2 <= y1) continue; // skip invalid

               tBBoxes.add(new float[]{x1, y1, x2, y2});
               tProbs.add((float) d.confidence());
               tNames.add(d.objectClass());
            }

            // Run tracker (2D only, masks/3D stay in your existing path)
            List<BoTSORTTrack> publishedTracks = botSort.update(frameW, frameH, tBBoxes, tProbs, tNames);
            latestPublishedTracks.put(yoloModel, publishedTracks);

            // TODO: temp hack
            synchronized (yoloDetectionResults)
            {
               if (yoloDetectionResults.containsKey(yoloModel))
                  yoloDetectionResults.remove(yoloModel).destroy();
               yoloDetectionResults.put(yoloModel, yoloResults);
            }

            if (newestColorImage.poll())
               newestColorImage.read().release();
            newestColorImage.set(bgrImage.get());

            SyncedYOLOv8ModelParameters modelParameters = parameters.getModelParameters().get(yoloModel.getName());

            // Create list of instant detections from results
            List<InstantDetection> yoloInstantDetections = new ArrayList<>();
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
               // Get the point cloud
               List<Point3D32> pointCloud = extractor.extractPointCloud(segmentedDepth);
               // Filter out outliers from the point cloud
               float outlierThreshold = modelParameters.getOutlierThresholds().getValueReadOnly(detection.objectClassID());
               pointCloud = YOLOv8Tools.filterOutliers(pointCloud, outlierThreshold, 128);
               // Get the centroid of the point cloud
               Point3D32 centroid = YOLOv8Tools.computeCentroidOfPointCloud(pointCloud, 128);
               if (centroid.containsNaN())
               {
                  erodedObjectMask.release();
                  segmentedDepth.release();
                  continue;
               }

               // Create an instant detection from data
               YOLOv8InstantDetection instantDetection = new YOLOv8InstantDetection(detection.objectClass(),
                                                                                    detection.confidence(),
                                                                                    new Pose3D(centroid, new RotationMatrix()),
                                                                                    erodedObjectMask.getAcquisitionTime(),
                                                                                    bgrImage,
                                                                                    erodedObjectMask,
                                                                                    depthImage,
                                                                                    detection.boundingBox(),
                                                                                    pointCloud);
               yoloInstantDetections.add(instantDetection);
               erodedObjectMask.release();
               segmentedDepth.release();
            }

            // Process callbacks
            if (!yoloInstantDetections.isEmpty())
               detectionConsumerCallbacks.forEach(callback -> callback.accept(yoloInstantDetections));

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

      for (YOLOv8DetectionList yoloResults : yoloDetectionResults.values())
         yoloResults.destroy();

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

   // Collect a copy of the latest published BoTSORT tracks across all models.
   private List<us.ihmc.perception.detections.botsortTracker.BoTSORTTrack> snapshotLatestTracks() {
      List<us.ihmc.perception.detections.botsortTracker.BoTSORTTrack> copy = new ArrayList<>();
      // latestPublishedTracks is a ConcurrentHashMap, but we'll still copy defensively
      latestPublishedTracks.values().forEach(list -> {
         if (list != null) copy.addAll(list);
      });
      return copy;
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
      synchronized (yoloDetectionResults)
      {
         List<YOLOv8Detection> allDetections = new ArrayList<>();
         yoloDetectionResults.values().forEach(allDetections::addAll);
         YOLOv8Tools.annotateImage(colorImage.getCpuImageMat(), resultMat, allDetections);
      }

      BytePointer annotatedImagePointer = new BytePointer();
      opencv_imgcodecs.imencode(".jpg", resultMat, annotatedImagePointer); // for some reason using CUDAImageEncoder broke YOLO's CUDNN

      ImageMessage imageMessage = new ImageMessage();
      PerceptionMessageTools.packImageMessage(colorImage, annotatedImagePointer, CompressionType.JPEG, imageMessage);
      annotatedImagePublisher.publish(imageMessage);

      // publish BoT-SORT tracked overlay to a separate topic
      List<BoTSORTTrack> tracks = snapshotLatestTracks();
      if (!tracks.isEmpty()) {
         Mat tracksMat = colorImage.getCpuImageMat().clone(); // start from raw color

         for (BoTSORTTrack t : tracks) {
            float[] b = t.getBbox();
            int x = Math.round(b[0]), y = Math.round(b[1]);
            int w = Math.round(b[2] - b[0]), h = Math.round(b[3] - b[1]);

            org.bytedeco.opencv.opencv_core.Rect rect =
                  new org.bytedeco.opencv.opencv_core.Rect(x, y, Math.max(1, w), Math.max(1, h));
            opencv_imgproc.rectangle(
                  tracksMat, rect,
                  new org.bytedeco.opencv.opencv_core.Scalar(0, 0, 255, 255), // red box
                  3, opencv_imgproc.LINE_4, 0);

            String label = String.format("#%d %s %.2f", t.getTargetId(), t.getName(), t.getScore());
            opencv_imgproc.putText(
                  tracksMat, label,
                  new Point(x, Math.max(0, y - 5)),
                  opencv_imgproc.FONT_HERSHEY_SIMPLEX, 0.7,
                  new org.bytedeco.opencv.opencv_core.Scalar(255, 255, 255, 255), // white text
                  2, opencv_imgproc.LINE_4, false);

            rect.close();
         }

         BytePointer tracksPtr = new BytePointer();
         opencv_imgcodecs.imencode(".jpg", tracksMat, tracksPtr);
         ImageMessage tracksMsg = new ImageMessage();
         PerceptionMessageTools.packImageMessage(colorImage, tracksPtr, CompressionType.JPEG, tracksMsg);
         trackedAnnotatedImagePublisher.publish(tracksMsg);

         tracksMat.close();
      }

      resultMat.close();
      colorImage.release();
   }

   private static void drawTracks(Mat img, List<BoTSORTTrack> tracks)
   {
      for (BoTSORTTrack t : tracks)
      {
         float[] b = t.getBbox();
         int x1 = Math.round(b[0]), y1 = Math.round(b[1]);
         int x2 = Math.round(b[2]), y2 = Math.round(b[3]);
         org.bytedeco.opencv.opencv_core.Rect r =
               new org.bytedeco.opencv.opencv_core.Rect(x1, y1, Math.max(1, x2 - x1), Math.max(1, y2 - y1));

         // green box
         opencv_imgproc.rectangle(img, r, new org.bytedeco.opencv.opencv_core.Scalar(0, 196, 0, 255), 3, opencv_imgproc.LINE_4, 0);

         String label = t.getName() + " #" + t.getTargetId();
         opencv_imgproc.putText(img, label,
                                new Point(x1, Math.max(0, y1 - 5)),
                                opencv_imgproc.FONT_HERSHEY_SIMPLEX, 0.7,
                                new org.bytedeco.opencv.opencv_core.Scalar(255, 255, 255, 255),
                                2, opencv_imgproc.LINE_AA, false);

         r.close();
      }
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
}
