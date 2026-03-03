package us.ihmc.perception.detections.yolo;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
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
   private final Object annotatedDetectionsLock = new Object();
   private final RepeatingTaskThread annotatedImagePublishedThread;
   private final TypedNotification<RawImage> newestColorImage = new TypedNotification<>();

   private final BoTSortTracker botSortTracker = new BoTSortTracker();

   private final Target2DTracker target2DTracker = new Target2DTracker(
         80,
         0.7f, 0.05f,
         10,
         50,
         0.05f, 0.4f,
         0.0f, 0.35f,
         100,
         new Target2DTracker.Weights()
   );

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

         //         annotatedImageDetections.remove(model);
      }

      while (modelIterator.hasNext())
      {
         YOLOv8Model model = modelIterator.next();

         if (parameters.getModelsToRun().getValue().contains(model.getName()))
         {
            runYOLODetection(model, colorImage, depthImage);
            return;
         }

         //         annotatedImageDetections.remove(model);
      }
   }

   /**
    * Non-blocking call to run YOLO on the provided images
    * @param colorImage BGR color image (or convertible), used for YOLO detection
    * @param depthImage 16UC1 depth image, used to get points of detected objects
    */
   private void runYOLODetection(YOLOv8Model yoloModel, RawImage colorImage, RawImage depthImage)
   {
      // If the queue is full, just drop this frame (keeps realtime behavior)
      if (taskQueue.remainingCapacity() <= 0)
         return;

      // IMPORTANT:
      // Retain references NOW (on the producer thread) so the runnable owns valid images
      // even if the caller releases immediately after runNextEnabledModel() returns.
      RawImage colorRef = colorImage.get();
      RawImage depthRef = depthImage.get();

      if (colorRef == null || depthRef == null)
      {
         // If either image is already dead, clean up the other retain (if any)
         if (colorRef != null) colorRef.release();
         if (depthRef != null) depthRef.release();
         return;
      }

      taskQueue.add(() ->
                    {
                       RawImage bgrImage = null;
                       try
                       {
                          // --- Convert to BGR8 on GPU (or keep as-is) ---
                          // YOLO expects BGR8. We create a new GPU mat and wrap it in a new RawImage
                          // that shares metadata (intrinsics, pose, timestamps).
                          GpuMat bgrMat = new GpuMat();
                          colorRef.getPixelFormat().convertToPixelFormat(colorRef.getGpuImageMat(), bgrMat, PixelFormat.BGR8);
                          bgrImage = colorRef.replaceImage(bgrMat, PixelFormat.BGR8);

                          // --- Run YOLO ---
                          YOLOv8DetectionList yoloResults = yoloModel.run(bgrImage);

                          // Publish newest color image for the annotated-image thread
                          // Store a RETAINED reference in newestColorImage
                          RawImage newest = bgrImage.get(); // ref++
                          if (newest != null)
                          {
                             // Release previous retained image (if any)
                             if (newestColorImage.poll())
                                newestColorImage.read().release();
                             newestColorImage.set(newest);
                          }

                          SyncedYOLOv8ModelParameters modelParameters = parameters.getModelParameters().get(yoloModel.getName());

                          // Build detections
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

                             // Segment depth
                             RawImage segmentedDepth = segmenter.removeBackground(depthRef, erodedObjectMask);
                             if (segmentedDepth == null)
                             {
                                erodedObjectMask.release();
                                continue;
                             }

                             // Point cloud
                             List<Point3D32> pointCloud = extractor.extractPointCloud(segmentedDepth);

                             // Filter outliers
                             float outlierThreshold = modelParameters.getOutlierThresholds().getValueReadOnly(detection.objectClassID());
                             List<Point3D32> filteredPoints = YOLOv8Tools.filterOutliers(pointCloud, outlierThreshold, 128);

                             if (!filteredPoints.isEmpty())
                             {
                                Point3D32 centroid = YOLOv8Tools.computeCentroidOfPointCloud(filteredPoints, 128);

                                // One instance for tracker + callbacks (may be destroyed by consumers)
                                YOLOv8InstantDetection detForTrackingAndCallbacks =
                                      new YOLOv8InstantDetection(detection.objectClass(),
                                                                 detection.confidence(),
                                                                 new Pose3D(centroid, new RotationMatrix()),
                                                                 erodedObjectMask.getAcquisitionTime(),
                                                                 bgrImage,
                                                                 erodedObjectMask,
                                                                 depthRef,
                                                                 detection.boundingBox(),
                                                                 filteredPoints);

                                // Separate instance ONLY for annotation thread (owned by YOLO executor)
                                YOLOv8InstantDetection detForAnnotation =
                                      new YOLOv8InstantDetection(detection.objectClass(),
                                                                 detection.confidence(),
                                                                 new Pose3D(centroid, new RotationMatrix()),
                                                                 erodedObjectMask.getAcquisitionTime(),
                                                                 bgrImage,
                                                                 erodedObjectMask,
                                                                 depthRef,
                                                                 detection.boundingBox(),
                                                                 filteredPoints);

                                trackableDetections.add(detForTrackingAndCallbacks);
                                yoloInstantDetections.add(detForTrackingAndCallbacks);

                                newAnnotatedImageDetections.add(detForAnnotation);
                             }

                             erodedObjectMask.release();
                             segmentedDepth.release();
                          }

                          // Track IDs
                          Mat frameForGmc = bgrImage.getCpuImageMat(); // CPU BGR
                          botSortTracker.update(frameForGmc, trackableDetections);

                          LogTools.info("tracked=" + botSortTracker.getTrackedCount() +
                                        " lost=" + botSortTracker.getLostCount() +
                                        " dets=" + trackableDetections.size());

                          // Copy track ids to annotation detections
                          for (int i = 0; i < trackableDetections.size(); i++)
                          {
                             newAnnotatedImageDetections.get(i).setTrackId(trackableDetections.get(i).getTrackId());
                          }

                          for (YOLOv8InstantDetection d : trackableDetections)
                             LogTools.info("Track: class=" + d.getDetectedObjectClass() + " id=" + d.getTrackId());

                          int frameW = bgrImage.getWidth();
                          int frameH = bgrImage.getHeight();

                          List<Target2DTracker.Obs> obs = new ArrayList<>();

                          Mat bgrCpu = bgrImage.getCpuImageMat();
                          Mat textureMap01 = TextureTools.computeTextureMap01(bgrCpu); // compute once

                          for (YOLOv8InstantDetection d : trackableDetections)
                          {
                             float[] bbox = { d.getX1(), d.getY1(), d.getX2(), d.getY2() };

                             RawImage maskRaw = d.getObjectMask();
                             Mat maskCpu = (maskRaw != null) ? maskRaw.getCpuImageMat() : null;

                             // --- ALIGN MASK RESOLUTION TO TEXTURE MAP RESOLUTION ---
                             Mat maskAligned = maskCpu;
                             Mat resized = null;

                             if (textureMap01 != null && maskCpu != null
                                 && (maskCpu.cols() != textureMap01.cols() || maskCpu.rows() != textureMap01.rows()))
                             {
                                resized = new Mat();
                                opencv_imgproc.resize(maskCpu,
                                                      resized,
                                                      new Size(textureMap01.cols(), textureMap01.rows()),
                                                      0, 0,
                                                      opencv_imgproc.INTER_NEAREST);
                                maskAligned = resized;
                             }

                             // --- TEXTURE ---
                             float texture01 = 0f;
                             if (textureMap01 != null && maskAligned != null)
                                texture01 = TextureTools.meanTextureInMask(textureMap01, maskAligned, bbox);

                             // IMPORTANT: release temporary resized mask (if created)
                             if (resized != null)
                                resized.release();

                             obs.add(new Target2DTracker.Obs(maskRaw, bbox, d.getTrackId(),
                                                             (float) d.getConfidence(), d.getObjectClass(), texture01));
                          }

                          if (textureMap01 != null) textureMap01.release();

                          List<Target2D> published = target2DTracker.update(frameW, frameH, obs);

                          // Optionally: attach published targetId back onto detections if you add a new field.
                          // Or: keep a map trackId->targetId for overlay/logging.

                          // Callbacks exactly once (FoundationPose, etc.)
                          detectionConsumerCallbacks.forEach(cb -> cb.accept(yoloInstantDetections));

                          // Swap annotated detections list (destroy previous)
                          synchronized (annotatedDetectionsLock)
                          {
                             List<YOLOv8InstantDetection> previous = this.annotatedImageDetections;
                             this.annotatedImageDetections = newAnnotatedImageDetections;
                             for (YOLOv8InstantDetection old : previous)
                                old.destroy();
                          }

                          // Cleanup YOLO outputs
                          yoloResults.destroy();
                       }
                       catch (Throwable t)
                       {
                          LogTools.error("YOLO task crashed", t);
                       }
                       finally
                       {
                          // Release bgrImage wrapper (and its GPU mat) if created
                          if (bgrImage != null)
                             bgrImage.release();

                          // Release the retained refs from the producer thread
                          colorRef.release();
                          depthRef.release();
                       }
                    });
   }

   public void destroy()
   {
      System.out.println("Destroying " + getClass().getSimpleName());
      updateThread.blockingKill();

      ros2Node.destroy();

      taskExecutorThread.kill();
      taskExecutorThread.interrupt();

      annotatedImagePublishedThread.kill();

      // release retained image (if any)
      if (newestColorImage.poll())
         newestColorImage.read().release();
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
         Runnable r = taskQueue.take();
         try
         {
            r.run();
         }
         catch (Throwable t)
         {
            LogTools.error("YOLO task executor caught exception", t);
         }
      }
      catch (InterruptedException ignored) {}
   }

   private void annotateAndPublishImage()
   {
      RawImage colorImage = newestColorImage.blockingPoll();
      if (colorImage == null)
         return;

      try
      {
         if (!annotatedImageDemanded.getAsBoolean())
         {
            LogTools.warn("YOLO annotated image NOT demanded; dropping frame");
            return;
         }

         Mat src = colorImage.getCpuImageMat();
         if (src == null || src.isNull())
         {
            LogTools.error("Annotated publisher got NULL CPU Mat. seq=" + colorImage.getSequenceNumber()
                           + " refs=" + colorImage.getReferenceCount());
            return;
         }

         Mat resultMat = new Mat();
         BytePointer annotatedImagePointer = null;

         try
         {
            synchronized (annotatedDetectionsLock)
            {
               YOLOv8Tools.annotateImage(src, resultMat, annotatedImageDetections);
            }

            annotatedImagePointer = new BytePointer();
            opencv_imgcodecs.imencode(".jpg", resultMat, annotatedImagePointer);

            ImageMessage imageMessage = new ImageMessage();
            PerceptionMessageTools.packImageMessage(colorImage, annotatedImagePointer, CompressionType.JPEG, imageMessage);
            annotatedImagePublisher.publish(imageMessage);
         }
         finally
         {
            if (annotatedImagePointer != null)
               annotatedImagePointer.deallocate(); // or close()

            resultMat.close();
         }
      }
      catch (Throwable t)
      {
         LogTools.error("YOLOAnnotatedImagePublisher crashed", t);
      }
      finally
      {
         colorImage.release();
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

   public Map<String, SyncedYOLOv8ModelParameters> getModelParameters()
   {
      return parameters.getModelParameters();
   }
}
