package us.ihmc.gdx.perception;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.environment.GDXEnvironmentBuilder;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.sensors.GDXHighLevelDepthSensorSimulator;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.gdx.ui.graphics.live.GDXROS2PointCloudVisualizer;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXGlobalVisualizersPanel;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;

public class GDXROS2PointCloudSensorDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");

   private GDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator;
   private final GDXPose3DGizmo sensorPoseGizmo = new GDXPose3DGizmo();
   private GDXEnvironmentBuilder environmentBuilder;
   private ImGuiGDXGlobalVisualizersPanel globalVisualizersPanel;
   private ROS2Node ros2Node;

   public GDXROS2PointCloudSensorDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "sensor_node");

            environmentBuilder = new GDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
            environmentBuilder.create();
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            environmentBuilder.loadEnvironment("DemoPullDoor.json");

            sensorPoseGizmo.create(baseUI.getPrimary3DPanel());
            sensorPoseGizmo.setResizeAutomatically(true);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(sensorPoseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(sensorPoseGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(sensorPoseGizmo, GDXSceneLevel.VIRTUAL);

            globalVisualizersPanel = new ImGuiGDXGlobalVisualizersPanel(false);
            globalVisualizersPanel.addVisualizer(new GDXROS2PointCloudVisualizer("Ouster Point Cloud",
                                                                                 ros2Node,
                                                                                 ROS2Tools.OUSTER_POINT_CLOUD,
                                                                                 2048 * 128,
                                                                                 1));
            globalVisualizersPanel.create();
            baseUI.getImGuiPanelManager().addPanel(globalVisualizersPanel);
            baseUI.getPrimaryScene().addRenderableProvider(globalVisualizersPanel, GDXSceneLevel.VIRTUAL);

            double publishRateHz = 20.0;
            double verticalFOV = 90.0;
            int imageWidth = 2048;
            int imageHeight = 128;
            double minRange = 0.105;
            double maxRange = 15.0;
            double noiseAmplitudeAtMinRange = 0.015;
            double noiseAmplitudeAtMaxRange = 0.05;
            boolean simulateL515Noise = false;
            highLevelDepthSensorSimulator = new GDXHighLevelDepthSensorSimulator("Ouster",
                                                                                 sensorPoseGizmo.getGizmoFrame(),
                                                                                 () -> 0L,
                                                                                 verticalFOV,
                                                                                 imageWidth,
                                                                                 imageHeight,
                                                                                 minRange,
                                                                                 maxRange,
                                                                                 noiseAmplitudeAtMinRange,
                                                                                 noiseAmplitudeAtMaxRange,
                                                                                 simulateL515Noise,
                                                                                 publishRateHz);
            highLevelDepthSensorSimulator.setupForROS2PointCloud(ros2Node, ROS2Tools.OUSTER_POINT_CLOUD);
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
            globalVisualizersPanel.update();

            for (GDXEnvironmentObject allObject : environmentBuilder.getAllObjects())
            {
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
      new GDXROS2PointCloudSensorDemo();
   }
}
