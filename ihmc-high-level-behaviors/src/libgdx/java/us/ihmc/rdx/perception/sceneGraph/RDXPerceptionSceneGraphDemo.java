package us.ihmc.rdx.perception.sceneGraph;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.OpenCVArUcoMarkerDetection;
import us.ihmc.perception.sceneGraph.PredefinedSceneNodeLibrary;
import us.ihmc.perception.sceneGraph.ROS2DetectableSceneNodesPublisher;
import us.ihmc.perception.sceneGraph.arUco.ArUcoSceneTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;
import us.ihmc.rdx.simulation.sensors.RDXSimulatedSensorFactory;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.ros2.ROS2Node;

public class RDXPerceptionSceneGraphDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private ROS2Node ros2Node;
   private ROS2Helper ros2Helper;
   private final RDXPose3DGizmo sensorPoseGizmo = new RDXPose3DGizmo();
   private RDXHighLevelDepthSensorSimulator simulatedCamera;
   private OpenCVArUcoMarkerDetection arUcoMarkerDetection;
   private PredefinedSceneNodeLibrary predefinedSceneNodeLibrary;
   private final ROS2DetectableSceneNodesPublisher detectableSceneObjectsPublisher = new ROS2DetectableSceneNodesPublisher();
   private RDXPerceptionSceneGraphUI perceptionSceneGraphUI;

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
            arUcoMarkerDetection.getDetectorParameters().markerBorderBits(2);
            arUcoMarkerDetection.setSourceImageForDetection(simulatedCamera.getLowLevelSimulator().getRGBA8888ColorImage());
            arUcoMarkerDetection.setCameraInstrinsics(simulatedCamera.getDepthCameraIntrinsics());

            predefinedSceneNodeLibrary = PredefinedSceneNodeLibrary.defaultObjects();
            perceptionSceneGraphUI = new RDXPerceptionSceneGraphUI(predefinedSceneNodeLibrary, ros2Helper);
            baseUI.getPrimaryScene().addRenderableProvider(perceptionSceneGraphUI::getRenderables);
            baseUI.getImGuiPanelManager().addPanel(perceptionSceneGraphUI.getPanel());
         }

         @Override
         public void render()
         {
            simulatedCamera.render(baseUI.getPrimaryScene());
            arUcoMarkerDetection.update();

            ArUcoSceneTools.updateLibraryPosesFromDetectionResults(arUcoMarkerDetection, predefinedSceneNodeLibrary);
            detectableSceneObjectsPublisher.publish(predefinedSceneNodeLibrary.getDetectableSceneNodes(), ros2Helper);

            perceptionSceneGraphUI.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            arUcoMarkerDetection.destroy();
            simulatedCamera.dispose();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXPerceptionSceneGraphDemo();
   }
}
