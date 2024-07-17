package us.ihmc.rdx.perception;

import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;
import us.ihmc.rdx.simulation.sensors.RDXSimulatedSensorFactory;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizersPanel;
import us.ihmc.rdx.ui.graphics.ros2.pointCloud.RDXROS2PointCloudVisualizer;
import us.ihmc.ros2.ROS2Node;

public class RDXROS2PointCloudSensorDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();

   private RDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator;
   private final RDXPose3DGizmo sensorPoseGizmo = new RDXPose3DGizmo();
   private RDXEnvironmentBuilder environmentBuilder;
   private RDXPerceptionVisualizersPanel perceptionVisualizerPanel;
   private ROS2Node ros2Node;

   public RDXROS2PointCloudSensorDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "sensor_node");

            environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
            environmentBuilder.create();
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            environmentBuilder.loadEnvironment("HarderTerrain.json");

            sensorPoseGizmo.create(baseUI.getPrimary3DPanel());
            sensorPoseGizmo.setResizeAutomatically(true);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(sensorPoseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(sensorPoseGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(sensorPoseGizmo, RDXSceneLevel.VIRTUAL);

            perceptionVisualizerPanel = new RDXPerceptionVisualizersPanel();

            RDXROS2PointCloudVisualizer ousterPointCloudVisualizer = new RDXROS2PointCloudVisualizer("Ouster Point Cloud",
                                                                                                     ros2Node,
                                                                                                     PerceptionAPI.OUSTER_POINT_CLOUD);
            perceptionVisualizerPanel.addVisualizer(ousterPointCloudVisualizer);

            perceptionVisualizerPanel.create();
            baseUI.getImGuiPanelManager().addPanel(perceptionVisualizerPanel);
            baseUI.getPrimaryScene().addRenderableProvider(perceptionVisualizerPanel);

            highLevelDepthSensorSimulator = RDXSimulatedSensorFactory.createOusterLidar(sensorPoseGizmo.getGizmoFrame(), () -> 0L);
            highLevelDepthSensorSimulator.setupForROS2PointCloud(ros2Node, PerceptionAPI.OUSTER_POINT_CLOUD);
            baseUI.getImGuiPanelManager().addPanel(highLevelDepthSensorSimulator);
            highLevelDepthSensorSimulator.setSensorEnabled(true);
            highLevelDepthSensorSimulator.setPublishPointCloudROS2(true);
            highLevelDepthSensorSimulator.setDebugCoordinateFrame(true);
            baseUI.getPrimaryScene().addRenderableProvider(highLevelDepthSensorSimulator::getRenderables);
         }

         @Override
         public void render()
         {
            highLevelDepthSensorSimulator.render(baseUI.getPrimaryScene());
            perceptionVisualizerPanel.update();

            for (RDXEnvironmentObject allObject : environmentBuilder.getAllObjects())
            {
               if (allObject.getRealisticModelInstance() != null)
                  allObject.getRealisticModelInstance().setDiffuseColor(highLevelDepthSensorSimulator.getPointColorFromPicker());
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
            environmentBuilder.destroy();
            highLevelDepthSensorSimulator.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXROS2PointCloudSensorDemo();
   }
}
