package us.ihmc.rdx.perception.sceneGraph;

import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2IOTopicQualifier;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerDetection;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerROS2Publisher;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraphPublisher;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraphSubscription;
import us.ihmc.perception.sceneGraph.arUco.ArUcoSceneTools;
import us.ihmc.perception.sceneGraph.multiBodies.door.DoorSceneNodeDefinitions;
import us.ihmc.perception.sceneGraph.rigidBodies.RigidBodySceneObjectDefinitions;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.perception.RDXOpenCVArUcoMarkerDetectionUI;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;
import us.ihmc.rdx.simulation.sensors.RDXSimulatedSensorFactory;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ArUcoMarkerPosesVisualizer;
import us.ihmc.rdx.ui.visualizers.RDXGlobalVisualizersPanel;
import us.ihmc.ros2.ROS2Node;

public class RDXPerceptionSceneGraphDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private ROS2Node ros2Node;
   private ROS2Helper ros2Helper;
   private RDXEnvironmentBuilder environmentBuilder;
   private final RDXPose3DGizmo sensorPoseGizmo = new RDXPose3DGizmo();
   private RDXHighLevelDepthSensorSimulator simulatedCamera;
   private RDXGlobalVisualizersPanel globalVisualizersUI;
   private OpenCVArUcoMarkerDetection arUcoMarkerDetection;
   private SceneGraph operatorSceneGraph;
   private SceneGraph onRobotSceneGraph;
   private OpenCVArUcoMarkerROS2Publisher arUcoMarkerPublisher;
   private ROS2SceneGraphSubscription sceneGraphSubscription;
   private final ROS2SceneGraphPublisher sceneGraphPublisher = new ROS2SceneGraphPublisher();
   private RDXPerceptionSceneGraphUI perceptionSceneGraphUI;
   private RDXOpenCVArUcoMarkerDetectionUI openCVArUcoMarkerDetectionUI;

   public RDXPerceptionSceneGraphDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "perception_scene_graph_demo");
            ros2Helper = new ROS2Helper(ros2Node);

            globalVisualizersUI = new RDXGlobalVisualizersPanel();
            baseUI.getImGuiPanelManager().addPanel(globalVisualizersUI);
            baseUI.getPrimaryScene().addRenderableProvider(globalVisualizersUI);

            environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
            environmentBuilder.create();
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            environmentBuilder.loadEnvironment("DoorsForArUcoTesting.json");

            sensorPoseGizmo.create(baseUI.getPrimary3DPanel());
            sensorPoseGizmo.setResizeAutomatically(true);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(sensorPoseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(sensorPoseGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(sensorPoseGizmo, RDXSceneLevel.VIRTUAL);

            simulatedCamera = RDXSimulatedSensorFactory.createBlackflyFisheye(sensorPoseGizmo.getGizmoFrame(), System::nanoTime);
            simulatedCamera.setSensorEnabled(true);
            simulatedCamera.setRenderColorVideoDirectly(true);
            baseUI.getPrimaryScene().addRenderableProvider(simulatedCamera::getRenderables);

            arUcoMarkerDetection = new OpenCVArUcoMarkerDetection();
            arUcoMarkerDetection.create(simulatedCamera.getSensorFrame());
            arUcoMarkerDetection.setSourceImageForDetection(simulatedCamera.getLowLevelSimulator().getRGBA8888ColorImage());
            arUcoMarkerDetection.setCameraInstrinsics(simulatedCamera.getDepthCameraIntrinsics());

            operatorSceneGraph = new SceneGraph();
            onRobotSceneGraph = new SceneGraph();

            RDXROS2ArUcoMarkerPosesVisualizer arUcoMarkerPosesVisualizer = new RDXROS2ArUcoMarkerPosesVisualizer("ArUco Marker Poses",
                                                                                                                 ros2Helper,
                                                                                                                 PerceptionAPI.ARUCO_MARKER_POSES);
            arUcoMarkerPosesVisualizer.setActive(true);
            globalVisualizersUI.addVisualizer(arUcoMarkerPosesVisualizer);

            arUcoMarkerPublisher = new OpenCVArUcoMarkerROS2Publisher(arUcoMarkerDetection,
                                                                      ros2Helper,
                                                                      onRobotSceneGraph.getArUcoMarkerIDsToSizesMap());

            sceneGraphSubscription = new ROS2SceneGraphSubscription(onRobotSceneGraph, ros2Helper, ROS2IOTopicQualifier.COMMAND);
            perceptionSceneGraphUI = new RDXPerceptionSceneGraphUI(operatorSceneGraph, ros2Helper, baseUI.getPrimary3DPanel());
            baseUI.getPrimaryScene().addRenderableProvider(perceptionSceneGraphUI::getRenderables);
            baseUI.getImGuiPanelManager().addPanel(perceptionSceneGraphUI.getPanel());

            openCVArUcoMarkerDetectionUI = new RDXOpenCVArUcoMarkerDetectionUI();
            openCVArUcoMarkerDetectionUI.create(arUcoMarkerDetection);
            baseUI.getImGuiPanelManager().addPanel(openCVArUcoMarkerDetectionUI.getMainPanel());

            globalVisualizersUI.create();
         }

         @Override
         public void render()
         {
            environmentBuilder.update();
            simulatedCamera.render(baseUI.getPrimaryScene());
            arUcoMarkerDetection.update();
            openCVArUcoMarkerDetectionUI.update();

            arUcoMarkerPublisher.update();

            sceneGraphSubscription.update();
            ArUcoSceneTools.updateLibraryPosesFromDetectionResults(arUcoMarkerDetection, onRobotSceneGraph);
            onRobotSceneGraph.update(sensorPoseGizmo.getGizmoFrame());
            sceneGraphPublisher.publish(onRobotSceneGraph, ros2Helper, ROS2IOTopicQualifier.STATUS);

            perceptionSceneGraphUI.update();

            globalVisualizersUI.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            globalVisualizersUI.destroy();
            arUcoMarkerDetection.destroy();
            simulatedCamera.dispose();
            baseUI.dispose();
            environmentBuilder.destroy();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXPerceptionSceneGraphDemo();
   }
}
