package us.ihmc.rdx.perception;

import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.heightMap.RemoteHeightMapUpdater;
import us.ihmc.perception.steppableRegions.RemoteSteppableRegionsUpdater;
import us.ihmc.perception.steppableRegions.SteppableRegionCalculatorParameters;
import us.ihmc.perception.steppableRegions.SteppableRegionCalculatorParametersReadOnly;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;
import us.ihmc.rdx.simulation.sensors.RDXSimulatedSensorFactory;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXInteractableReferenceFrame;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.ros2.RDXHeightMapVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2PointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXSteppableRegionsVisualizer;
import us.ihmc.rdx.ui.graphics.RDXGeneralToolsPanel;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.RealtimeROS2Node;

public class RDXSteppableRegionCalculatorDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private RDXHighLevelDepthSensorSimulator ousterLidarSimulator;
   private RDXInteractableReferenceFrame robotInteractableReferenceFrame;
   private RDXPose3DGizmo ousterPoseGizmo = new RDXPose3DGizmo();
   private RDXEnvironmentBuilder environmentBuilder;

   private final RemoteHeightMapUpdater heightMap;
   private final RemoteSteppableRegionsUpdater steppableRegionsUpdater;
   private final RDXRemoteHeightMapPanel heightMapUI;
   private final RDXSteppableRegionsPanel steppableRegionsUI;
   private final RDXGeneralToolsPanel globalVisualizersUI;

   public RDXSteppableRegionCalculatorDemo()
   {
      CommunicationMode ros2CommunicationMode = CommunicationMode.INTERPROCESS;

      RealtimeROS2Node realtimeRos2Node = ROS2Tools.createRealtimeROS2Node(ros2CommunicationMode.getPubSubImplementation(), "simulation_ui_realtime");
      ROS2Node ros2Node = ROS2Tools.createROS2Node(ros2CommunicationMode.getPubSubImplementation(), "simulation_ui_realtime");
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

      heightMap = new RemoteHeightMapUpdater("", ReferenceFrame::getWorldFrame, realtimeRos2Node);
      heightMap.getParameters().setMaxZ(1.5);

      heightMapUI = new RDXRemoteHeightMapPanel(ros2Helper);

      SteppableRegionCalculatorParametersReadOnly defaultSteppableParameters = new SteppableRegionCalculatorParameters();
      steppableRegionsUpdater = new RemoteSteppableRegionsUpdater(ros2Helper, defaultSteppableParameters);
      steppableRegionsUpdater.start();
      steppableRegionsUI = new RDXSteppableRegionsPanel(ros2Helper, defaultSteppableParameters);

      baseUI.getImGuiPanelManager().addPanel(heightMapUI.getPanel());

      // Configure the height map visualizer
      globalVisualizersUI = new RDXGeneralToolsPanel();

      RDXHeightMapVisualizer heightMapVisualizer = new RDXHeightMapVisualizer();
      heightMapVisualizer.setActive(true);
      RDXSteppableRegionsVisualizer steppableRegionsVisualizer = new RDXSteppableRegionsVisualizer("Steppable Regions");
      steppableRegionsVisualizer.setActive(true);

      baseUI.getImGuiPanelManager().addPanel(globalVisualizersUI);
      baseUI.getPrimaryScene().addRenderableProvider(globalVisualizersUI);

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            heightMapUI.create();
            steppableRegionsUI.create();
            globalVisualizersUI.create();
            baseUI.create();

            environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
            environmentBuilder.create();
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            environmentBuilder.loadEnvironment("DemoPullDoor.json");

            heightMapVisualizer.create();
            steppableRegionsVisualizer.create();
            //            baseUI.getImGuiPanelManager().addPanel(heightMapVisualizer.getPanel());
            globalVisualizersUI.addVisualizer(heightMapVisualizer);
            globalVisualizersUI.addVisualizer(steppableRegionsVisualizer);

            steppableRegionsUI.getEnabled().set(true);

            new IHMCROS2Callback<>(ros2Node, PerceptionAPI.HEIGHT_MAP_OUTPUT, message ->
            {
               heightMapVisualizer.acceptHeightMapMessage(message);
               heightMapUI.acceptHeightMapMessage(message);

               steppableRegionsUpdater.submitLatestHeightMapMessage(message);
            });

            steppableRegionsVisualizer.setUpForNetworking(ros2Node);
            steppableRegionsUI.setUpForNetworking(ros2Node);

            robotInteractableReferenceFrame = new RDXInteractableReferenceFrame();
            robotInteractableReferenceFrame.create(ReferenceFrame.getWorldFrame(), 0.15, baseUI.getPrimary3DPanel());
            robotInteractableReferenceFrame.getTransformToParent().getTranslation().add(2.2, 1.25, 1.0);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(robotInteractableReferenceFrame::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(robotInteractableReferenceFrame::getVirtualRenderables, RDXSceneLevel.VIRTUAL);
            ousterPoseGizmo = new RDXPose3DGizmo(robotInteractableReferenceFrame.getRepresentativeReferenceFrame());
            ousterPoseGizmo.create(baseUI.getPrimary3DPanel());
            ousterPoseGizmo.setResizeAutomatically(false);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(ousterPoseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(ousterPoseGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(ousterPoseGizmo, RDXSceneLevel.VIRTUAL);
            ousterPoseGizmo.getTransformToParent().appendPitchRotation(Math.toRadians(60.0));

            ousterLidarSimulator = RDXSimulatedSensorFactory.createOusterLidar(ousterPoseGizmo.getGizmoFrame(), () -> 0L);
            ousterLidarSimulator.setupForROS2PointCloud(ros2Node, PerceptionAPI.OUSTER_LIDAR_SCAN);
            ousterLidarSimulator.setSensorEnabled(true);
            ousterLidarSimulator.setRenderPointCloudDirectly(true);
            ousterLidarSimulator.setPublishPointCloudROS2(true);
            ousterLidarSimulator.setDebugCoordinateFrame(false);

            RDXROS2PointCloudVisualizer ousterPointCloudVisualizer = new RDXROS2PointCloudVisualizer("Ouster Point Cloud",
                                                                                                     ros2Node,
                                                                                                     PerceptionAPI.OUSTER_LIDAR_SCAN);
            ousterPointCloudVisualizer.setSubscribed(true);
            globalVisualizersUI.addVisualizer(ousterPointCloudVisualizer);

            baseUI.getImGuiPanelManager().addPanel(ousterLidarSimulator);
            baseUI.getPrimaryScene().addRenderableProvider(ousterLidarSimulator::getRenderables);

            baseUI.getImGuiPanelManager().addPanel(steppableRegionsUI.getBasePanel());

            realtimeRos2Node.spin();
            heightMap.start();
         }

         @Override
         public void render()
         {
            ousterLidarSimulator.render(baseUI.getPrimaryScene());

            heightMap.update();
            heightMapVisualizer.update();
            steppableRegionsVisualizer.update();

            globalVisualizersUI.update();
            heightMapUI.update();
            steppableRegionsUI.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
            environmentBuilder.destroy();
            realtimeRos2Node.destroy();
            ros2Node.destroy();
            globalVisualizersUI.destroy();
            heightMapVisualizer.destroy();
            heightMapUI.destroy();
            steppableRegionsUpdater.destroy();
            steppableRegionsUI.destroy();
            ousterLidarSimulator.dispose();

            super.dispose();
         }
      });

      Runtime.getRuntime().addShutdownHook(new Thread(baseUI::dispose));
   }

   public static void main(String[] args)
   {
      new RDXSteppableRegionCalculatorDemo();
   }
}
