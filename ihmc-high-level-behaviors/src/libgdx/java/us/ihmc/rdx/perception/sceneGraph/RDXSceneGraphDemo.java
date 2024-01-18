package us.ihmc.rdx.perception.sceneGraph;

import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerDetector;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerDetectionResults;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerROS2Publisher;
import us.ihmc.perception.sceneGraph.SceneObjectDefinitions;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.perception.sceneGraph.arUco.ArUcoSceneTools;
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
import us.ihmc.rdx.ui.graphics.RDXGlobalVisualizersPanel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.ros2.ROS2Node;
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
   private RDXGlobalVisualizersPanel globalVisualizersUI;
   private OpenCVArUcoMarkerDetector arUcoMarkerDetector;
   private OpenCVArUcoMarkerDetectionResults arUcoMarkerDetectionResults;
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

            onRobotSceneGraph = new ROS2SceneGraph(ros2Helper);

            RDXROS2ArUcoMarkerPosesVisualizer arUcoMarkerPosesVisualizer = new RDXROS2ArUcoMarkerPosesVisualizer("ArUco Marker Poses",
                                                                                                                 ros2Helper,
                                                                                                                 PerceptionAPI.ARUCO_MARKER_POSES);
            arUcoMarkerPosesVisualizer.setActive(true);
            globalVisualizersUI.addVisualizer(arUcoMarkerPosesVisualizer);

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

            globalVisualizersUI.create();
         }

         @Override
         public void render()
         {
            boolean runPerception = perceptionThottler.run();

            environmentBuilder.update();
            simulatedCamera.render(baseUI.getPrimaryScene());
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

            globalVisualizersUI.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            globalVisualizersUI.destroy();
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
