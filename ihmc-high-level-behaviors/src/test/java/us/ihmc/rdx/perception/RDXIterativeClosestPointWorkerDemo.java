package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.IterativeClosestPointWorker;
import us.ihmc.perception.OpenCLPointCloudExtractor;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizerPanel;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ColoredPointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.ZEDColorDepthImagePublisher;
import us.ihmc.sensors.ZEDColorDepthImageRetriever;
import us.ihmc.tools.thread.RestartableThread;

import java.text.DecimalFormat;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

public class RDXIterativeClosestPointWorkerDemo
{
   private static final int MAX_ENVIRONMENT_SIZE = 1000;
   private static final int CORRESPONDENCES = 1000;

   private final OpenCLManager openCLManager = new OpenCLManager();
   private final OpenCLPointCloudExtractor pointCloudExtractor = new OpenCLPointCloudExtractor(openCLManager);
   private final Random random = new Random(System.nanoTime());

   private final ROS2Node node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "icp_worker_demo");
   private final ROS2Helper ros2Helper = new ROS2Helper(node);
   private final ZEDColorDepthImageRetriever zedImageRetriever;
   private final ZEDColorDepthImagePublisher zedImagePublisher;
   private RawImage zedDepthImage;
   private RawImage zedLeftColorImage;
   private IterativeClosestPointWorker icpWorker = new IterativeClosestPointWorker(MAX_ENVIRONMENT_SIZE, CORRESPONDENCES, random);

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final RDXPerceptionVisualizerPanel perceptionVisualizerPanel = new RDXPerceptionVisualizerPanel();
   private RDXReferenceFrameGraphic referenceFrameGraphic;

   private ModelInstance mousePickSphere;
   FramePoint3D pickFramePoint = new FramePoint3D();

   private final RDXPointCloudRenderer icpBoxRenderer = new RDXPointCloudRenderer();
   private List<Point3D32> objectPointCloud;
   private final RecyclingArrayList<Point3D32> icpBoxPointCloud = new RecyclingArrayList<>(Point3D32::new);

   private final RDXPointCloudRenderer segmentedPointCloudRenderer = new RDXPointCloudRenderer();
   private final RDXPointCloudRenderer correspondingObjectPointCloudRenderer = new RDXPointCloudRenderer();
   private final RDXPointCloudRenderer correspondingMeasurementPointCloudRenderer = new RDXPointCloudRenderer();
   private final RecyclingArrayList<Point3D32> segmentedPtCld = new RecyclingArrayList<>(Point3D32::new);
   private final RecyclingArrayList<Point3D32> correspondingObjectPtCld = new RecyclingArrayList<>(Point3D32::new);
   private final RecyclingArrayList<Point3D32> correspondingMeasurementPtCld = new RecyclingArrayList<>(Point3D32::new);

   private boolean mouseTrackingToggle = true;

   private final Pose3D shapeInputPose = new Pose3D();
   private PrimitiveRigidBodyShape shape = PrimitiveRigidBodyShape.BOX;
   private final ImInt shapeIndex = new ImInt();
   private final String[] shapeValues = new String[PrimitiveRigidBodyShape.values().length];
   private final ImFloat depth = new ImFloat(0.19f);
   private final ImFloat width = new ImFloat(0.405f);
   private final ImFloat height = new ImFloat(0.31f);
   private final ImFloat xRadius = new ImFloat(0.1f);
   private final ImFloat yRadius = new ImFloat(0.1f);
   private final ImFloat zRadius = new ImFloat(0.1f);
   private final ImInt numberOfShapeSamples = new ImInt(1000);
   private final ImInt numberOfCorespondences = new ImInt(1000);
   private final ImFloat segmentationRadius = new ImFloat(0.2f);
   private final ImInt numberOfIterations = new ImInt(1);
   private double icpGuiICPRunTimeInSeconds;

   public RDXIterativeClosestPointWorkerDemo()
   {
      zedImageRetriever = new ZEDColorDepthImageRetriever(0,
                                                          ReferenceFrame::getWorldFrame,
                                                          new ROS2DemandGraphNode(ros2Helper, PerceptionAPI.REQUEST_ZED_DEPTH),
                                                          new ROS2DemandGraphNode(ros2Helper, PerceptionAPI.REQUEST_ZED_COLOR));
      zedImagePublisher = new ZEDColorDepthImagePublisher(PerceptionAPI.ZED2_COLOR_IMAGES, PerceptionAPI.ZED2_DEPTH);

      PrimitiveRigidBodyShape[] shapeArray = new PrimitiveRigidBodyShape[PrimitiveRigidBodyShape.values().length];
      Arrays.stream(PrimitiveRigidBodyShape.values()).toList().toArray(shapeArray);

      for (int i = 0; i < PrimitiveRigidBodyShape.values().length; ++i)
      {
         shapeValues[i] = shapeArray[i].name();
      }

      RestartableThread zedPublishThread = new RestartableThread("ZedPublish", this::readAndPublishZED);
      zedPublishThread.start();

      RestartableThread uiThread = new RestartableThread("UI", this::runUI);
      uiThread.start();
   }

   private void calculateICPTime(long time1, long time2)
   {
      long timeDiffNanos = time2-time1;
      icpGuiICPRunTimeInSeconds = Conversions.nanosecondsToSeconds(timeDiffNanos);
   }

   private void readAndPublishZED()
   {
      zedDepthImage = zedImageRetriever.getLatestRawDepthImage();
      zedLeftColorImage = zedImageRetriever.getLatestRawColorImage(RobotSide.LEFT);

      if (zedDepthImage != null && !zedDepthImage.isEmpty())
         icpWorker.setEnvironmentPointCloud(pointCloudExtractor.extractPointCloud(zedDepthImage));

      if (mouseTrackingToggle)
         shapeInputPose.set(pickFramePoint, icpWorker.getResultPose().getOrientation());
      else
         shapeInputPose.set(icpWorker.getResultPose());

      icpWorker.setTargetPoint(shapeInputPose.getPosition());
      icpWorker.useProvidedTargetPoint(mouseTrackingToggle);
      icpWorker.setSegmentSphereRadius(segmentationRadius.get());

      long startTimeNanos = System.nanoTime();
      boolean success = icpWorker.runICP(numberOfIterations.get());
      long stopTimeNanos = System.nanoTime();
      calculateICPTime(startTimeNanos, stopTimeNanos);
      if (success)
         ros2Helper.publish(PerceptionAPI.ICP_RESULT, icpWorker.getResult());

      List<? extends Point3DReadOnly> segmentedPointCloud = icpWorker.getSegmentedPointCloud();
      segmentedPtCld.clear();
      if (segmentedPointCloud != null && !segmentedPointCloud.isEmpty())
      {
         for (int i = 0; i < MAX_ENVIRONMENT_SIZE * 10 && i < segmentedPointCloud.size(); ++i)
         {
            Point3D32 newPoint = segmentedPtCld.add();
            newPoint.set(segmentedPointCloud.get(i));
         }
         segmentedPointCloudRenderer.setPointsToRender(segmentedPtCld, Color.GRAY);
      }
      correspondingObjectPtCld.clear();
      List<Point3DReadOnly> correspondingObjectPointCLoud = icpWorker.getCorrespondingObjectPoints();
      if (correspondingObjectPointCLoud != null && !correspondingObjectPointCLoud.isEmpty())
      {
         for (int i = 0; i < MAX_ENVIRONMENT_SIZE * 10 && i < correspondingObjectPointCLoud.size(); ++i)
         {
            Point3D32 newPoint = correspondingObjectPtCld.add();
            newPoint.set(correspondingObjectPointCLoud.get(i));
         }
         correspondingObjectPointCloudRenderer.setPointsToRender(correspondingObjectPtCld, Color.RED);
      }
      correspondingMeasurementPtCld.clear();
      List<Point3DReadOnly> correspondingMeasurementPointCloud = icpWorker.getCorrespondingMeasurementPoints();
      if (correspondingMeasurementPointCloud != null && !correspondingMeasurementPointCloud.isEmpty())
      {
         for (int i = 0; i < MAX_ENVIRONMENT_SIZE * 10 && i < correspondingMeasurementPointCloud.size(); ++i)
         {
            Point3D32 newPoint = correspondingMeasurementPtCld.add();
            newPoint.set(correspondingMeasurementPointCloud.get(i));
         }
         correspondingMeasurementPointCloudRenderer.setPointsToRender(correspondingMeasurementPtCld, Color.BLUE);
      }


      referenceFrameGraphic.setPoseInWorldFrame(icpWorker.getResultPose());

      zedImagePublisher.setNextColorImage(zedLeftColorImage.get(), RobotSide.LEFT);
      zedImagePublisher.setNextGpuDepthImage(zedDepthImage.get());

      zedLeftColorImage.release();
      zedDepthImage.release();
   }

   private void runUI()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {

         @Override
         public void create()
         {
            mousePickSphere = RDXModelBuilder.createSphere(0.03f, Color.RED);
            baseUI.getPrimaryScene().addRenderableProvider(mousePickSphere, RDXSceneLevel.VIRTUAL);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(this::calculatePickPoint);

            icpBoxRenderer.create(MAX_ENVIRONMENT_SIZE);
            baseUI.getPrimaryScene().addRenderableProvider(icpBoxRenderer, RDXSceneLevel.VIRTUAL);

            segmentedPointCloudRenderer.create(MAX_ENVIRONMENT_SIZE * 10);
            correspondingMeasurementPointCloudRenderer.create(MAX_ENVIRONMENT_SIZE * 10);
            correspondingObjectPointCloudRenderer.create(MAX_ENVIRONMENT_SIZE * 10);
            baseUI.getPrimaryScene().addRenderableProvider(segmentedPointCloudRenderer);
            baseUI.getPrimaryScene().addRenderableProvider(correspondingMeasurementPointCloudRenderer);
            baseUI.getPrimaryScene().addRenderableProvider(correspondingObjectPointCloudRenderer);

            RDXROS2ImageMessageVisualizer zedDepthImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED2 Depth Image",
                                                                                                      DomainFactory.PubSubImplementation.FAST_RTPS,
                                                                                                      PerceptionAPI.ZED2_DEPTH);
            perceptionVisualizerPanel.addVisualizer(zedDepthImageVisualizer, PerceptionAPI.REQUEST_ZED_COLOR);

            RDXROS2ColoredPointCloudVisualizer zedPointCloudVisualizer = new RDXROS2ColoredPointCloudVisualizer("ZED2 Colored Point Cloud",
                                                                                                                DomainFactory.PubSubImplementation.FAST_RTPS,
                                                                                                                PerceptionAPI.ZED2_DEPTH,
                                                                                                                PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT));
            perceptionVisualizerPanel.addVisualizer(zedPointCloudVisualizer, PerceptionAPI.REQUEST_ZED_POINT_CLOUD);

            baseUI.getImGuiPanelManager().addPanel("Settings", this::renderSettings);

            referenceFrameGraphic = new RDXReferenceFrameGraphic(0.3);
            baseUI.getPrimaryScene().addRenderableProvider(referenceFrameGraphic);

            baseUI.getImGuiPanelManager().addPanel(perceptionVisualizerPanel);
            baseUI.create();
            baseUI.getPrimaryScene().addRenderableProvider(perceptionVisualizerPanel);
            perceptionVisualizerPanel.create();
         }

         @Override
         public void render()
         {
            if (ImGui.isMouseClicked(ImGuiMouseButton.Right))
            {
               mouseTrackingToggle = !mouseTrackingToggle;
            }

            icpBoxPointCloud.clear();
            objectPointCloud = icpWorker.getObjectPointCloud();
            if (objectPointCloud != null && !objectPointCloud.isEmpty())
            {
               for (int i = 0; i < MAX_ENVIRONMENT_SIZE && i < objectPointCloud.size(); ++i)
               {
                  Point3D32 newPoint = icpBoxPointCloud.add();
                  newPoint.set(objectPointCloud.get(i));
               }
               icpBoxRenderer.setPointsToRender(icpBoxPointCloud, Color.GOLD);
            }

            icpBoxRenderer.updateMesh();

            segmentedPointCloudRenderer.updateMesh();
            correspondingMeasurementPointCloudRenderer.updateMesh();
            correspondingObjectPointCloudRenderer.updateMesh();

            perceptionVisualizerPanel.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            zedImageRetriever.destroy();
            perceptionVisualizerPanel.destroy();
            baseUI.dispose();
         }

         private void renderSettings()
         {
            if (ImGui.combo("Shape", shapeIndex, shapeValues))
            {
               shape = PrimitiveRigidBodyShape.valueOf(shapeValues[shapeIndex.get()]);
               icpWorker = new IterativeClosestPointWorker(shape,
                                                           new Vector3D(depth.get(), width.get(), height.get()),
                                                           new Vector3D(xRadius.get(), yRadius.get(), zRadius.get()),
                                                           MAX_ENVIRONMENT_SIZE,
                                                           CORRESPONDENCES,
                                                           new FramePose3D(ReferenceFrame.getWorldFrame(), pickFramePoint, new RotationMatrix()),
                                                           random);
            }
            ImGui.sliderFloat("Depth", depth.getData(), 0.0f, 1.0f);
            ImGui.sliderFloat("Width", width.getData(), 0.0f, 1.0f);
            ImGui.sliderFloat("Height", height.getData(), 0.0f, 1.0f);
            ImGui.sliderFloat("xRadius", xRadius.getData(), 0.0f, 1.0f);
            ImGui.sliderFloat("yRadius", yRadius.getData(), 0.0f, 1.0f);
            ImGui.sliderFloat("zRadius", zRadius.getData(), 0.0f, 1.0f);
            ImGui.sliderInt("Num Shape Samples", numberOfShapeSamples.getData(), 0, 10000);
            ImGui.sliderInt("Num Correspondences", numberOfCorespondences.getData(), 0, 10000);
            if (ImGui.button("Apply Size"))
            {
               icpWorker.changeSize(new Vector3D(depth.get(), width.get(), height.get()),
                                    new Vector3D(xRadius.get(), yRadius.get(), zRadius.get()),
                                    numberOfShapeSamples.get());
               icpWorker.setNumberOfCorrespondences(numberOfCorespondences.get());
            }
            ImGui.sliderFloat("Segmentation Radius", segmentationRadius.getData(), 0.0f, 1.0f);
            ImGui.sliderInt("Number of Iterations", numberOfIterations.getData(), 1, 10);

            ImGui.separator();

            DecimalFormat df = new DecimalFormat("#.###");
            DecimalFormat tf = new DecimalFormat("#.#########");

            ImGui.text("ICP Time: " + tf.format(icpGuiICPRunTimeInSeconds));

            ImGui.text("Input Centroid: " + df.format(shapeInputPose.getX()) + " y: " + df.format(shapeInputPose.getY()) + " z: " + df.format(shapeInputPose.getZ()));
            ImGui.text("Res Centroid: " + df.format(icpWorker.getResultPose().getX()) + " y: " + df.format(icpWorker.getResultPose().getY()) + " z: " + df.format(icpWorker.getResultPose().getZ()));
            ImGui.text("diff Centroid: " + df.format(shapeInputPose.getX() - icpWorker.getResultPose().getX()) + " y: " + df.format(shapeInputPose.getY()- icpWorker.getResultPose().getY()) + " z: " + df.format(shapeInputPose.getZ()-icpWorker.getResultPose().getZ()));
         }

         private void calculatePickPoint(us.ihmc.rdx.input.ImGui3DViewInput input)
         {
            pickFramePoint.set(input.getPickPointInWorld());
            LibGDXTools.toLibGDX(pickFramePoint, mousePickSphere.transform);
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXIterativeClosestPointWorkerDemo();
   }
}
