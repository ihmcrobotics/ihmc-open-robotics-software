package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import imgui.ImGui;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.OpenCLDepthImageSegmenter;
import us.ihmc.perception.OpenCLPointCloudExtractor;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.YOLOv8.YOLOv8DetectableObject;
import us.ihmc.perception.YOLOv8.YOLOv8DetectionResults;
import us.ihmc.perception.YOLOv8.YOLOv8ObjectDetector;
import us.ihmc.perception.iterativeClosestPoint.IterativeClosestPointWorker;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.realsense.RealsenseConfiguration;
import us.ihmc.perception.realsense.RealsenseDeviceManager;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizerPanel;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ColoredPointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.RealsenseColorDepthImagePublisher;
import us.ihmc.sensors.RealsenseColorDepthImageRetriever;
import us.ihmc.sensors.ZEDColorDepthImagePublisher;
import us.ihmc.sensors.ZEDColorDepthImageRetriever;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;
import us.ihmc.tools.thread.Throttler;

import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

public class RDXYOLOv8IterativeClosestPointDemo
{
   private static final String CSV_FILE_NAME = "ihmc_mug_points.csv";
   private static final boolean USE_CUSTOM_OBJECT = false;
   private static final YOLOv8DetectableObject OBJECT_TYPE = YOLOv8DetectableObject.KEYBOARD;

   private static final boolean USE_REALSENSE = true;
   private static final String REALSENSE_NUMBER = "215122254074";

   private static final Random random = new Random();

   private final OpenCLManager openCLManager = new OpenCLManager();
   private final OpenCLPointCloudExtractor extractor = new OpenCLPointCloudExtractor(openCLManager);
   private final OpenCLDepthImageSegmenter segmenter = new OpenCLDepthImageSegmenter(openCLManager);

   private final ROS2Node node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "yolo_icp_demo");
   private final ROS2Helper ros2Helper = new ROS2Helper(node);
   private final ZEDColorDepthImageRetriever zedImageRetriever;
   private final ZEDColorDepthImagePublisher zedImagePublisher;
   private final RealsenseDeviceManager realsenseManager = new RealsenseDeviceManager();
   private final RealsenseColorDepthImageRetriever realsenseRetriever;
   private final RealsenseColorDepthImagePublisher realsensePublisher;

   private final YOLOv8ObjectDetector yoloObjectDetector = new YOLOv8ObjectDetector();

   private final IterativeClosestPointWorker icpWorker;

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final RDXPerceptionVisualizerPanel perceptionVisualizerPanel = new RDXPerceptionVisualizerPanel();
   private final RDXPointCloudRenderer segmentedPointCloudRenderer = new RDXPointCloudRenderer();
   private final RecyclingArrayList<Point3D32> segmentedPointCloud = new RecyclingArrayList<>(Point3D32::new);
   private final RDXPointCloudRenderer icpObjectPointCloudRenderer = new RDXPointCloudRenderer();
   private RDXReferenceFrameGraphic centroidGraphic;

   private final ImInt numICPIterations = new ImInt(1);

   private final Vector3D objectLengths = new Vector3D(0.135, 0.43, 0.015);
   private final Vector3D objectRadii = new Vector3D(0.09, 0.2, 0.23);
   private final ImFloat xLength = new ImFloat(objectLengths.getX32());
   private final ImFloat yLength = new ImFloat(objectLengths.getY32());
   private final ImFloat zLength = new ImFloat(objectLengths.getZ32());
   private final ImFloat xRadius = new ImFloat(objectRadii.getX32());
   private final ImFloat yRadius = new ImFloat(objectRadii.getY32());
   private final ImFloat zRadius = new ImFloat(objectRadii.getZ32());

   private final ImFloat confidenceThreshold = new ImFloat(0.5f);
   private final ImFloat nmsThreshold = new ImFloat(0.1f);
   private final ImFloat maskThreshold = new ImFloat(0.0f);
   private final ImFloat outlierRejectionThreshold = new ImFloat(10.0f);
   private final ImInt erosionValue = new ImInt(3);

   public RDXYOLOv8IterativeClosestPointDemo()
   {
      icpWorker = new IterativeClosestPointWorker(OBJECT_TYPE.getCorrespondingShape(), objectLengths, objectRadii, 1000, 1000, new Pose3D(), random);
      icpWorker.useProvidedTargetPoint(false);
      icpWorker.setSegmentSphereRadius(Double.MAX_VALUE);
      if (USE_CUSTOM_OBJECT)
      {
         WorkspaceResourceDirectory directory = new WorkspaceResourceDirectory(YOLOv8DetectableObject.class, "/yoloICPPointClouds/");
         WorkspaceResourceFile file = new WorkspaceResourceFile(directory, CSV_FILE_NAME);

         icpWorker.setDetectionShape(PrimitiveRigidBodyShape.CUSTOM, file.getFilesystemFile().toFile());
      }

      zedImageRetriever = new ZEDColorDepthImageRetriever(0,
                                                          ReferenceFrame::getWorldFrame,
                                                          new ROS2DemandGraphNode(ros2Helper, PerceptionAPI.REQUEST_ZED_DEPTH),
                                                          new ROS2DemandGraphNode(ros2Helper, PerceptionAPI.REQUEST_ZED_COLOR));
      zedImagePublisher = new ZEDColorDepthImagePublisher(PerceptionAPI.ZED2_COLOR_IMAGES, PerceptionAPI.ZED2_DEPTH);

      realsenseRetriever = new RealsenseColorDepthImageRetriever(realsenseManager,
                                                                 REALSENSE_NUMBER,
                                                                 RealsenseConfiguration.D455_COLOR_720P_DEPTH_720P_30HZ,
                                                                 ReferenceFrame::getWorldFrame,
                                                                 new ROS2DemandGraphNode(ros2Helper, PerceptionAPI.REQUEST_REALSENSE_POINT_CLOUD));
      realsensePublisher = new RealsenseColorDepthImagePublisher(PerceptionAPI.D455_DEPTH_IMAGE, PerceptionAPI.D455_COLOR_IMAGE);

      if (USE_REALSENSE)
         realsenseRetriever.start();
      else
         zedImageRetriever.start();



      ThreadTools.startAThread(this::runUI, getClass().getSimpleName() + "UI");

      Throttler throttler = new Throttler();
      throttler.setFrequency(30.0);
      while (true)
      {
         throttler.waitAndRun();
         RawImage depthImage = USE_REALSENSE ? realsenseRetriever.getLatestRawDepthImage() : zedImageRetriever.getLatestRawDepthImage();
         RawImage colorImage = USE_REALSENSE ? realsenseRetriever.getLatestRawColorImage() : zedImageRetriever.getLatestRawColorImage(RobotSide.LEFT);

         YOLOv8DetectionResults results = yoloObjectDetector.runOnImage(colorImage, confidenceThreshold.get(), nmsThreshold.get());
         RawImage objectMask = results.getSegmentationMatrixForObject(OBJECT_TYPE, maskThreshold.get());
         if (objectMask != null)
         {
            RawImage segmentedDepth = segmenter.removeBackground(depthImage, objectMask, erosionValue.get());

            List<Point3DReadOnly> ptcld = extractor.extractPointCloud(segmentedDepth);

            Point3D centroid = new Point3D();
            double stdDev = doMath(ptcld, 500, true, centroid);

            ptcld = ptcld.parallelStream().filter(point ->
            {
               Vector3D zVector = new Vector3D(point);
               zVector.sub(centroid);
               zVector.scale(1.0 / stdDev);
               return zVector.norm() < outlierRejectionThreshold.get();
            }).collect(Collectors.toList());

            if (segmentedPointCloud != null)
               segmentedPointCloud.clear();
            for (int i = 0; i < Short.MAX_VALUE && i < ptcld.size(); ++i)
            {
               Point3D32 point = segmentedPointCloud.add();
               point.set(ptcld.get(i));
            }
            segmentedPointCloudRenderer.setPointsToRender(segmentedPointCloud, Color.GREEN);

            icpWorker.setEnvironmentPointCloud(segmentedPointCloud);

            segmentedDepth.release();
            objectMask.release();
         }

         if (icpWorker.runICP(numICPIterations.get()))
         {
            icpObjectPointCloudRenderer.setPointsToRender(icpWorker.getObjectPointCloud(), Color.YELLOW);
            centroidGraphic.setPoseInWorldFrame(icpWorker.getResultPose());
         }

         if (USE_REALSENSE)
         {
            realsensePublisher.setNextColorImage(colorImage.get());
            realsensePublisher.setNextDepthImage(depthImage.get());
         }
         else
         {
            zedImagePublisher.setNextColorImage(colorImage.get(), RobotSide.LEFT);
            zedImagePublisher.setNextGpuDepthImage(depthImage.get());
         }

         depthImage.release();
         colorImage.release();
         results.destroy();
      }
   }

   private void runUI()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            segmentedPointCloudRenderer.create(Short.MAX_VALUE);
            baseUI.getPrimaryScene().addRenderableProvider(segmentedPointCloudRenderer);

            icpObjectPointCloudRenderer.create(Short.MAX_VALUE);
            baseUI.getPrimaryScene().addRenderableProvider(icpObjectPointCloudRenderer);

            centroidGraphic = new RDXReferenceFrameGraphic(0.3);
            baseUI.getPrimaryScene().addRenderableProvider(centroidGraphic);

            RDXROS2ImageMessageVisualizer colorImageVisualizer = new RDXROS2ImageMessageVisualizer("Color Image",
                                                                                                      DomainFactory.PubSubImplementation.FAST_RTPS,
                                                                                                      USE_REALSENSE ?
                                                                                                            PerceptionAPI.D455_COLOR_IMAGE :
                                                                                                            PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT));
            perceptionVisualizerPanel.addVisualizer(colorImageVisualizer,
                                                    USE_REALSENSE ? PerceptionAPI.REQUEST_REALSENSE_POINT_CLOUD : PerceptionAPI.REQUEST_ZED_COLOR);
            RDXROS2ColoredPointCloudVisualizer pointCloudVisualizer = new RDXROS2ColoredPointCloudVisualizer("Colored Point Cloud",
                                                                                                                DomainFactory.PubSubImplementation.FAST_RTPS,
                                                                                                                USE_REALSENSE ?
                                                                                                                      PerceptionAPI.D455_DEPTH_IMAGE :
                                                                                                                      PerceptionAPI.ZED2_DEPTH,
                                                                                                                USE_REALSENSE ?
                                                                                                                      PerceptionAPI.D455_COLOR_IMAGE :
                                                                                                                      PerceptionAPI.ZED2_COLOR_IMAGES.get(
                                                                                                                            RobotSide.LEFT));
            perceptionVisualizerPanel.addVisualizer(pointCloudVisualizer,
                                                    USE_REALSENSE ? PerceptionAPI.REQUEST_REALSENSE_POINT_CLOUD : PerceptionAPI.REQUEST_ZED_POINT_CLOUD);

            baseUI.getImGuiPanelManager().addPanel(perceptionVisualizerPanel);
            baseUI.create();
            baseUI.getPrimaryScene().addRenderableProvider(perceptionVisualizerPanel);
            perceptionVisualizerPanel.create();
            baseUI.getImGuiPanelManager().addPanel("Settings", this::renderSettings);
         }

         @Override
         public void render()
         {
            segmentedPointCloudRenderer.updateMesh();
            icpObjectPointCloudRenderer.updateMesh();
            perceptionVisualizerPanel.update();
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderSettings()
         {
            ImGui.sliderInt("numIterations", numICPIterations.getData(), 1, 10);

            ImGui.newLine();

            ImGui.sliderFloat("xLength", xLength.getData(), 0.0f, 1.0f);
            ImGui.sliderFloat("yLength", yLength.getData(), 0.0f, 1.0f);
            ImGui.sliderFloat("zLength", zLength.getData(), 0.0f, 1.0f);
            ImGui.sliderFloat("xRadius", xRadius.getData(), 0.0f, 1.0f);
            ImGui.sliderFloat("yRadius", yRadius.getData(), 0.0f, 1.0f);
            ImGui.sliderFloat("zRadius", zRadius.getData(), 0.0f, 1.0f);
            if (ImGui.button("Apply Size"))
            {
               objectLengths.set(xLength.get(), yLength.get(), zLength.get());
               objectRadii.set(xRadius.get(), yRadius.get(), zRadius.get());
               icpWorker.changeSize(objectLengths, objectRadii, 500);
            }

            ImGui.sliderFloat("confidenceThreshold", confidenceThreshold.getData(), 0.0f, 1.0f);
            ImGui.sliderFloat("nmsThreshold", nmsThreshold.getData(), 0.0f, 1.0f);
            ImGui.sliderFloat("maskThreshold", maskThreshold.getData(), -1.0f, 1.0f);
            ImGui.sliderFloat("outlierRejectionThreshold", outlierRejectionThreshold.getData(), 0.0f, 50.0f);
            ImGui.sliderInt("erosionValue", erosionValue.getData(), 0, 20);
         }

         @Override
         public void dispose()
         {
            yoloObjectDetector.destroy();
            zedImageRetriever.destroy();
            zedImagePublisher.destroy();
            realsenseManager.deleteContext();
            realsenseRetriever.destroy();
            realsensePublisher.destroy();
            perceptionVisualizerPanel.destroy();
            baseUI.dispose();
         }
      });
   }

   private double doMath(List<? extends Point3DReadOnly> pointCloud, Point3DBasics centroidToPack)
   {
      return doMath(pointCloud, pointCloud.size(), false, centroidToPack);
   }

   /**
    * Given a point cloud, computes the centroid and variance of the points
    * @param pointCloud       The list of points used for calculations
    * @param maxComputations  Maximum number of points to use for the computation. First N points in the list will be used.
    * @param shuffle          Whether to shuffle the point cloud before computations. Can be used to find approximate values with N points
    * @param centroidToPack   Point object into which the centroid will be packed
    * @return The standard deviation of the points
    */
   private double doMath(List<? extends Point3DReadOnly> pointCloud, int maxComputations, boolean shuffle, Point3DBasics centroidToPack)
   {
      if (shuffle)
         Collections.shuffle(pointCloud);

      Vector3D sumVector = new Vector3D(0.0, 0.0, 0.0);
      Vector3D squaredSumVector = new Vector3D(0.0, 0.0, 0.0);
      int numberOfComputations = Math.min(pointCloud.size(), maxComputations);

      pointCloud.parallelStream().limit(numberOfComputations).forEach(point ->
      {
         sumVector.add(point);
         squaredSumVector.add((point.getX() * point.getX()), (point.getY() * point.getY()), (point.getZ() * point.getZ()));
      });

      centroidToPack.set(sumVector);
      centroidToPack.scale(1.0 / numberOfComputations);

      Vector3D meanSquaredVector = new Vector3D(centroidToPack);
      meanSquaredVector.scale(meanSquaredVector.getX(), meanSquaredVector.getY(), meanSquaredVector.getZ());

      Vector3D varianceVector = new Vector3D(squaredSumVector);
      varianceVector.scale(1.0 / numberOfComputations);
      varianceVector.sub(meanSquaredVector);

      return Math.sqrt(varianceVector.getX() + varianceVector.getY() + varianceVector.getZ());
   }

   public static void main(String[] args)
   {
      new RDXYOLOv8IterativeClosestPointDemo();
   }
}
