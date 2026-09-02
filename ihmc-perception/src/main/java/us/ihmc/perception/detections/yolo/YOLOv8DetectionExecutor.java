package us.ihmc.perception.detections.yolo;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Size;
import perception_msgs.ImageMessage;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.cuda.CUDADepthImageSegmenter;
import us.ihmc.perception.cuda.CUDAPointCloudExtractor;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.tools.PerceptionMessageTools;

import java.net.URL;
import java.util.ArrayList;
import java.util.Arrays;
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
   private final ROS2Node ros2Node;
   private final boolean ownsRos2Node;

   private final CUDAPointCloudExtractor extractor;
   private final CUDADepthImageSegmenter segmenter;

   private final Map<String, YOLOv8Model> availableModels = new LinkedHashMap<>();
   private List<YOLOv8InstantDetection> annotatedImageDetections = new ArrayList<>();
   private Iterator<YOLOv8Model> modelIterator;

   private final SyncedYOLOv8ExecutorParameters parameters;
   private final RepeatingTaskThread updateThread;

   private final List<Consumer<List<InstantDetection>>> detectionConsumerCallbacks = new ArrayList<>();

   private List<AnnotatedTarget2D> annotatedTargets = new ArrayList<>();
   private final Object annotatedTargetsLock = new Object();

   private final ROS2Publisher<ImageMessage> annotatedImagePublisher;
   private final BooleanSupplier annotatedImageDemanded;
   private final Object annotatedDetectionsLock = new Object();
   private final RepeatingTaskThread annotatedImagePublishedThread;
   private final TypedNotification<RawImage> newestColorImage = new TypedNotification<>();

   private final BoTSORTTracker botSortTracker = new BoTSORTTracker();

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
   private final RepeatingTaskThread taskExecutorThread =
         new RepeatingTaskThread("YOLOExecutor", this::executeTasks, DefaultExceptionHandler.RUNTIME_EXCEPTION);

   /**
    * New-style constructor: executor owns its own ROS2 node.
    */
   public YOLOv8DetectionExecutor(ROS2PeerClockOffsetEstimator peerClockEstimator, BooleanSupplier annotatedImageDemanded)
   {
      this(new ROS2Node("yolo_detection_manager"), true, peerClockEstimator, annotatedImageDemanded);
   }

   /**
    * Old-style constructor: caller provides the ROS2 node.
    * Kept for backward compatibility.
    */
   public YOLOv8DetectionExecutor(ROS2Node ros2Node,
                                  ROS2PeerClockOffsetEstimator peerClockEstimator,
                                  BooleanSupplier annotatedImageDemanded)
   {
      this(ros2Node, false, peerClockEstimator, annotatedImageDemanded);
   }

   private YOLOv8DetectionExecutor(ROS2Node ros2Node,
                                   boolean ownsRos2Node,
                                   ROS2PeerClockOffsetEstimator peerClockEstimator,
                                   BooleanSupplier annotatedImageDemanded)
   {
      this.ros2Node = ros2Node;
      this.ownsRos2Node = ownsRos2Node;
      this.annotatedImageDemanded = annotatedImageDemanded;
      this.annotatedImagePublisher = ros2Node.createPublisher(PerceptionAPI.YOLO_ANNOTATED_IMAGE);

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

      for (URL yoloModelDirectory : YOLOv8Tools.getYOLOModelDirectories())
      {
         YOLOv8Model model = new YOLOv8Model(yoloModelDirectory);

         LogTools.info("Loaded YOLOv8 model: {}", model.getName());
         LogTools.info("\t\t\tClasses: {}", model.getDetectableObjectCount());

         availableModels.put(model.getName(), model);
      }

      if (availableModels.isEmpty())
         LogTools.error("No YOLO models found. YOLO will not run.");

      parameters = new SyncedYOLOv8ExecutorParameters(ros2Node, crdtInfo);
      parameters.setAvailableModels(availableModels.values());
      parameters.requestSendFullData();

      updateThread = new RepeatingTaskThread(getClass().getSimpleName() + "Updater", this::update);
      updateThread.setFrequencyLimit(10.0);
      updateThread.setDaemon(true);
      updateThread.startRepeating();

      taskQueue = new ArrayBlockingQueue<>(4);
      taskExecutorThread.setDaemon(true);
      taskExecutorThread.startRepeating();

      annotatedImagePublishedThread =
            new RepeatingTaskThread("YOLOAnnotatedImagePublisher", this::annotateAndPublishImage, DefaultExceptionHandler.RUNTIME_EXCEPTION);
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

   public Map<String, SyncedYOLOv8ModelParameters> getModelParameters()
   {
      return parameters.getModelParameters();
   }

   public SyncedYOLOv8ExecutorParameters getParameters()
   {
      return parameters;
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

   /**
    * Backward-compatible alias for older code.
    */
   public void runNextModel(RawImage colorImage, RawImage depthImage)
   {
      runNextEnabledModel(colorImage, depthImage);
   }

   /**
    * Current preferred name.
    */
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
      }

      while (modelIterator != null && modelIterator.hasNext())
      {
         YOLOv8Model model = modelIterator.next();

         if (parameters.getModelsToRun().getValue().contains(model.getName()))
         {
            runYOLODetection(model, colorImage, depthImage);
            return;
         }
      }
   }

   /**
    * Non-blocking call to run YOLO on the provided images.
    */
   private void runYOLODetection(YOLOv8Model yoloModel, RawImage colorImage, RawImage depthImage)
   {
      if (taskQueue.remainingCapacity() <= 0)
         return;

      RawImage colorRef = colorImage.get();
      RawImage depthRef = depthImage.get();

      if (colorRef == null || depthRef == null)
      {
         if (colorRef != null)
            colorRef.release();
         if (depthRef != null)
            depthRef.release();
         return;
      }

      taskQueue.add(() ->
                    {
                       RawImage bgrImage = null;

                       try
                       {
                          GpuMat bgrMat = new GpuMat();
                          colorRef.getPixelFormat().convertToPixelFormat(colorRef.getGpuImageMat(), bgrMat, PixelFormat.BGR8);
                          bgrImage = colorRef.replaceImage(bgrMat, PixelFormat.BGR8);

                          YOLOv8DetectionList yoloResults = yoloModel.run(bgrImage);

                          SyncedYOLOv8ModelParameters modelParameters = parameters.getModelParameters().get(yoloModel.getName());
                          if (modelParameters == null)
                          {
                             yoloResults.destroy();
                             return;
                          }

                          List<InstantDetection> yoloInstantDetections = new ArrayList<>();
                          List<YOLOv8InstantDetection> trackableDetections = new ArrayList<>();
                          List<YOLOv8InstantDetection> newAnnotatedImageDetections = new ArrayList<>();

                          for (YOLOv8Detection detection : yoloResults)
                          {
                             RawImage objectMask = detection.mask();

                             int erosionKernelRadius = modelParameters.getErosionKernelRadii().getValueReadOnly(detection.objectClassID());
                             Mat erodedMask = new Mat(objectMask.getHeight(), objectMask.getWidth(), objectMask.getOpenCVType());
                             opencv_imgproc.erode(objectMask.getCpuImageMat(),
                                                  erodedMask,
                                                  opencv_imgproc.getStructuringElement(opencv_imgproc.CV_SHAPE_RECT,
                                                                                       new Size(2 * erosionKernelRadius + 1, 2 * erosionKernelRadius + 1),
                                                                                       new Point(erosionKernelRadius, erosionKernelRadius)));
                             RawImage erodedObjectMask = objectMask.replaceImage(erodedMask);
                             objectMask.release();

                             RawImage segmentedDepth = segmenter.removeBackground(depthRef, erodedObjectMask);
                             if (segmentedDepth == null)
                             {
                                erodedObjectMask.release();
                                continue;
                             }

                             List<Point3D32> pointCloud = extractor.extractPointCloud(segmentedDepth);

                             float outlierThreshold = modelParameters.getOutlierThresholds().getValueReadOnly(detection.objectClassID());
                             List<Point3D32> filteredPoints = YOLOv8Tools.filterOutliers(pointCloud, outlierThreshold, 128);

                             if (!filteredPoints.isEmpty())
                             {
                                Point3D32 centroid = YOLOv8Tools.computeCentroidOfPointCloud(filteredPoints, 128);

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

                          Mat frameForGmc = bgrImage.getCpuImageMat();
                          botSortTracker.update(frameForGmc, trackableDetections);

                          for (int i = 0; i < trackableDetections.size(); i++)
                             newAnnotatedImageDetections.get(i).setTrackId(trackableDetections.get(i).getTrackId());

                          int frameW = bgrImage.getWidth();
                          int frameH = bgrImage.getHeight();

                          List<Target2DTracker.Obs> obs = new ArrayList<>();

                          Mat bgrCpu = bgrImage.getCpuImageMat();
                          Mat textureMap01 = TextureTools.computeTextureMap01(bgrCpu);

                          for (YOLOv8InstantDetection d : trackableDetections)
                          {
                             float[] bbox = {d.getX1(), d.getY1(), d.getX2(), d.getY2()};

                             RawImage maskRaw = d.getObjectMask();
                             Mat maskCpu = maskRaw != null ? maskRaw.getCpuImageMat() : null;

                             Mat maskAligned = maskCpu;
                             Mat resized = null;

                             if (textureMap01 != null && maskCpu != null
                                 && (maskCpu.cols() != textureMap01.cols() || maskCpu.rows() != textureMap01.rows()))
                             {
                                resized = new Mat();
                                opencv_imgproc.resize(maskCpu,
                                                      resized,
                                                      new Size(textureMap01.cols(), textureMap01.rows()),
                                                      0,
                                                      0,
                                                      opencv_imgproc.INTER_NEAREST);
                                maskAligned = resized;
                             }

                             float texture01 = 0.0f;
                             if (textureMap01 != null && maskAligned != null)
                                texture01 = TextureTools.meanTextureInMask(textureMap01, maskAligned, bbox);

                             if (resized != null)
                                resized.release();

                             obs.add(new Target2DTracker.Obs(maskRaw,
                                                             bbox,
                                                             d.getTrackId(),
                                                             (float) d.getConfidence(),
                                                             d.getObjectClass(),
                                                             texture01));
                          }

                          if (textureMap01 != null)
                             textureMap01.release();

                          List<Target2D> published = target2DTracker.update(frameW, frameH, obs);

                          List<AnnotatedTarget2D> newAnnotatedTargets = new ArrayList<>(published.size());
                          for (Target2D t : published)
                          {
                             float[] bb = t.latestBbox != null ? Arrays.copyOf(t.latestBbox, 4) : null;
                             RawImage mask = t.latestMask != null ? t.latestMask.get() : null;

                             newAnnotatedTargets.add(new AnnotatedTarget2D(
                                   t.targetId,
                                   t.lastTrackId,
                                   t.name,
                                   t.score,
                                   bb,
                                   mask));
                          }

                          synchronized (annotatedTargetsLock)
                          {
                             List<AnnotatedTarget2D> prev = annotatedTargets;
                             annotatedTargets = newAnnotatedTargets;
                             for (AnnotatedTarget2D at : prev)
                                at.destroy();
                          }

                          /*
                           * Make the RGB frame available only after its corresponding annotation targets have been generated.
                           */

                          RawImage newest = bgrImage.get();

                          if (newest != null)
                          {
                             if (newestColorImage.poll())
                                newestColorImage.read().release();

                             newestColorImage.set(newest);
                          }

                          detectionConsumerCallbacks.forEach(cb -> cb.accept(yoloInstantDetections));

                          synchronized (annotatedDetectionsLock)
                          {
                             List<YOLOv8InstantDetection> previous = this.annotatedImageDetections;
                             this.annotatedImageDetections = newAnnotatedImageDetections;
                             for (YOLOv8InstantDetection old : previous)
                                old.destroy();
                          }

                          yoloResults.destroy();
                       }
                       catch (Throwable t)
                       {
                          LogTools.error("YOLO task crashed", t);
                       }
                       finally
                       {
                          if (bgrImage != null)
                             bgrImage.release();

                          colorRef.release();
                          depthRef.release();
                       }
                    });
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
      catch (InterruptedException ignored)
      {
      }
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
            LogTools.error("Annotated publisher got NULL CPU Mat. seq={} refs={}",
                           colorImage.getSequenceNumber(),
                           colorImage.getReferenceCount());
            return;
         }

         Mat resultMat = new Mat();
         BytePointer annotatedImagePointer = null;

         try
         {
            src.copyTo(resultMat);

            synchronized (annotatedTargetsLock)
            {
               YOLOv8Tools.annotateTargets(resultMat, annotatedTargets);
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
               annotatedImagePointer.deallocate();

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
      parameters.checkModifiedAndUpdate();

      if (parameters.isModified())
      {
         parameters.getModelParameters().forEach((modelName, modelParameters) ->
                                                 {
                                                    YOLOv8Model model = availableModels.get(modelName);
                                                    if (model != null)
                                                       modelParameters.applyToModel(model);
                                                 });
      }
   }

   public void destroy()
   {
      System.out.println("Destroying " + getClass().getSimpleName());

      updateThread.blockingKill();

      taskExecutorThread.kill();
      taskExecutorThread.interrupt();

      annotatedImagePublishedThread.kill();

      if (newestColorImage.poll())
         newestColorImage.read().release();
      newestColorImage.set(null);

      for (YOLOv8Model yoloModel : availableModels.values())
         yoloModel.destroy();

      extractor.close();
      segmenter.close();

      parameters.close();

      if (ownsRos2Node)
         ros2Node.close();

      System.out.println("Destroyed " + getClass().getSimpleName());
   }
}