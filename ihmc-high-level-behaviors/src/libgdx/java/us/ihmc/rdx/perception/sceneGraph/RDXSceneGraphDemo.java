package us.ihmc.rdx.perception.sceneGraph;

import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerDetectionResults;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerDetector;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerROS2Publisher;
import us.ihmc.perception.sceneGraph.SceneObjectDefinitions;
import us.ihmc.perception.sceneGraph.arUco.ArUcoSceneTools;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorSceneNodeDefinitions;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.perception.sceneGraph.yolo.YOLOv8DetectionManager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.perception.RDXOpenCVArUcoMarkerDetectionUI;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;
import us.ihmc.rdx.simulation.sensors.RDXSimulatedSensorFactory;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizersPanel;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ArUcoMarkerPosesVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.rdx.ui.interactable.RDXInteractableObject;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.RestartableThrottledThread;
import us.ihmc.tools.thread.Throttler;

/**
 * A self contained demo and development environment for our scene graph functionality.
 */
public class RDXSceneGraphDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private ROS2Node ros2Node;
   private ROS2Helper ros2Helper;
   private RDXEnvironmentBuilder environmentBuilder;
   private final RDXPose3DGizmo sensorPoseGizmo = new RDXPose3DGizmo("SimulatedSensor");
   private RDXHighLevelDepthSensorSimulator simulatedCamera;
   private RDXPerceptionVisualizersPanel perceptionVisualizerPanel;
   private OpenCVArUcoMarkerDetector arUcoMarkerDetector;
   private OpenCVArUcoMarkerDetectionResults arUcoMarkerDetectionResults;
   private YOLOv8DetectionManager yolov8DetectionManager;
   private ROS2SceneGraph onRobotSceneGraph;
   private OpenCVArUcoMarkerROS2Publisher arUcoMarkerPublisher;
   private ReferenceFrameLibrary referenceFrameLibrary;
   private RDXSceneGraphUI sceneGraphUI;
   private RDXOpenCVArUcoMarkerDetectionUI openCVArUcoMarkerDetectionUI;
   /** Simulate an update rate more similar to what it would be on the robot. */
   private final Throttler perceptionThottler = new Throttler().setFrequency(30.0);

   public RDXSceneGraphDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create(RDXSceneLevel.VIRTUAL, RDXSceneLevel.MODEL, RDXSceneLevel.GROUND_TRUTH);

            PubSubImplementation pubSubImplementation = PubSubImplementation.FAST_RTPS;
            ros2Node = ROS2Tools.createROS2Node(pubSubImplementation, "perception_scene_graph_demo");
            ros2Helper = new ROS2Helper(ros2Node);

            perceptionVisualizerPanel = new RDXPerceptionVisualizersPanel();
            baseUI.getImGuiPanelManager().addPanel(perceptionVisualizerPanel);
            baseUI.getPrimaryScene().addRenderableProvider(perceptionVisualizerPanel);

            RDXROS2ImageMessageVisualizer yoloAnnotatedImageVisualizer = new RDXROS2ImageMessageVisualizer("YOLOv8 Annotated Image",
                                                                                                           pubSubImplementation,
                                                                                                           PerceptionAPI.YOLO_ANNOTATED_IMAGE);
            yoloAnnotatedImageVisualizer.setActive(true);
            perceptionVisualizerPanel.addVisualizer(yoloAnnotatedImageVisualizer);

            environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
            environmentBuilder.create();
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            environmentBuilder.loadEnvironment("DoorsForArUcoTesting.json");

            sensorPoseGizmo.create(baseUI.getPrimary3DPanel());
            sensorPoseGizmo.setResizeAutomatically(true);
            sensorPoseGizmo.getTransformToParent().getTranslation().setZ(0.7);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(sensorPoseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(sensorPoseGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(sensorPoseGizmo, RDXSceneLevel.VIRTUAL);

            simulatedCamera = RDXSimulatedSensorFactory.createBlackflyFisheye(sensorPoseGizmo.getGizmoFrame(), System::nanoTime);
            simulatedCamera.setSensorEnabled(true);
            simulatedCamera.setRenderColorVideoDirectly(true);
            baseUI.getPrimaryScene().addRenderableProvider(simulatedCamera::getRenderables);

            arUcoMarkerDetector = new OpenCVArUcoMarkerDetector();
            arUcoMarkerDetector.setSourceImageForDetection(simulatedCamera.getLowLevelSimulator().getRGBA8888ColorImage());
            arUcoMarkerDetector.setCameraInstrinsics(simulatedCamera.getDepthCameraIntrinsics());
            arUcoMarkerDetectionResults = new OpenCVArUcoMarkerDetectionResults();

            RDXInteractableObject interactableKnob = new RDXInteractableObject(baseUI);
            interactableKnob.load(DoorSceneNodeDefinitions.DOOR_KNOB_VISUAL_MODEL_FILE_PATH,
                                  DoorSceneNodeDefinitions.DOOR_KNOB_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
            interactableKnob.getSelectablePose3DGizmo().setSelected(true);
            baseUI.getPrimaryScene().addRenderableProvider(interactableKnob.getModelInstance(), RDXSceneLevel.GROUND_TRUTH);
            baseUI.getPrimaryScene().addRenderableProvider(interactableKnob.getSelectablePose3DGizmo()::getVirtualRenderables, RDXSceneLevel.VIRTUAL);

            RestartableThrottledThread yoloDetectionThread = new RestartableThrottledThread("YOLODetection", 8.0, () ->
            {
               if (yolov8DetectionManager == null)
               {
                  yolov8DetectionManager = new YOLOv8DetectionManager(ros2Helper, yoloAnnotatedImageVisualizer::isActive);
                  yolov8DetectionManager.setRobotFrame(sensorPoseGizmo.getGizmoFrame());
               }

               RawImage rawColorImageBGR;
               RawImage rawDepthImageDiscretized;
               synchronized (simulatedCamera)
               {
                  rawColorImageBGR = simulatedCamera.createRawColorImageBGR();
                  rawDepthImageDiscretized = simulatedCamera.createRawDepthImageDiscretized();
               }
               yolov8DetectionManager.runYOLODetection(rawColorImageBGR, rawDepthImageDiscretized);
            });
            yoloDetectionThread.start();

            onRobotSceneGraph = new ROS2SceneGraph(ros2Helper);

            RDXROS2ArUcoMarkerPosesVisualizer arUcoMarkerPosesVisualizer = new RDXROS2ArUcoMarkerPosesVisualizer("ArUco Marker Poses",
                                                                                                                 ros2Helper,
                                                                                                                 PerceptionAPI.ARUCO_MARKER_POSES);
            arUcoMarkerPosesVisualizer.setActive(true);
            perceptionVisualizerPanel.addVisualizer(arUcoMarkerPosesVisualizer);

            arUcoMarkerPublisher = new OpenCVArUcoMarkerROS2Publisher(arUcoMarkerDetectionResults,
                                                                      ros2Helper,
                                                                      SceneObjectDefinitions.ARUCO_MARKER_SIZES,
                                                                      simulatedCamera.getSensorFrame());

            referenceFrameLibrary = new ReferenceFrameLibrary();

            sceneGraphUI = new RDXSceneGraphUI(ros2Helper, baseUI.getPrimary3DPanel());
            baseUI.getPrimaryScene().addRenderableProvider(sceneGraphUI::getRenderables);
            baseUI.getImGuiPanelManager().addPanel(sceneGraphUI.getPanel());

            openCVArUcoMarkerDetectionUI = new RDXOpenCVArUcoMarkerDetectionUI();
            openCVArUcoMarkerDetectionUI.create(arUcoMarkerDetector.getDetectorParameters());
            baseUI.getImGuiPanelManager().addPanel(openCVArUcoMarkerDetectionUI.getMainPanel());

            perceptionVisualizerPanel.create();
         }

         @Override
         public void render()
         {
            boolean runPerception = perceptionThottler.run();

            environmentBuilder.update();
            synchronized (simulatedCamera)
            {
               simulatedCamera.render(baseUI.getPrimaryScene());
            }
            if (runPerception)
            {
               arUcoMarkerDetector.update();
               openCVArUcoMarkerDetectionUI.copyOutputData(arUcoMarkerDetector);
               arUcoMarkerDetectionResults.copyOutputData(arUcoMarkerDetector);
            }
            openCVArUcoMarkerDetectionUI.update();

            if (runPerception)
            {
               arUcoMarkerPublisher.update();

               onRobotSceneGraph.updateSubscription();
               ArUcoSceneTools.updateSceneGraph(arUcoMarkerDetectionResults, simulatedCamera.getSensorFrame(), onRobotSceneGraph);
               onRobotSceneGraph.updateOnRobotOnly(sensorPoseGizmo.getGizmoFrame());
               onRobotSceneGraph.updatePublication();
            }

            sceneGraphUI.update();

            perceptionVisualizerPanel.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            yolov8DetectionManager.destroy();
            perceptionVisualizerPanel.destroy();
            simulatedCamera.dispose();
            baseUI.dispose();
            environmentBuilder.destroy();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXSceneGraphDemo();
   }
}
