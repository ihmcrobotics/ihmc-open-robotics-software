package us.ihmc.gdx.ui;

import boofcv.struct.calib.CameraPinholeBrown;
import com.badlogic.gdx.math.Matrix4;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.environment.GDXEnvironmentBuilderPanel;
import us.ihmc.gdx.simulation.environment.object.objects.GDXL515SensorObject;
import us.ihmc.gdx.simulation.sensors.GDXHighLevelDepthSensorSimulator;
import us.ihmc.behaviors.tools.PlanarRegionSLAMMapper;
import us.ihmc.behaviors.tools.perception.PeriodicPlanarRegionPublisher;
import us.ihmc.gdx.ui.graphics.live.*;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXGlobalVisualizersPanel;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.RosTools;

import java.net.URISyntaxException;

public class GDXPerceptionVisualizerUI
{
    private final PlanarRegionSLAMMapper realsensePlanarRegionSLAM = new PlanarRegionSLAMMapper();
    private PlanarRegionsList regionsUpdate = new PlanarRegionsList();
//    private final GDXHighLevelDepthSensorSimulator simulatedDepthSensor;

    private final GDXImGuiBasedUI baseUI;
    private final ImGuiMapSenseConfigurationPanel mapsenseConfigurationUI;
    private final ImGuiGDXGlobalVisualizersPanel globalVisualizersUI;
    private final GDXEnvironmentBuilderPanel environmentBuilderUI;

    private final RigidBodyTransform depthSensorTransform = new RigidBodyTransform();
    private final Matrix4 gdxSensorTransform = new Matrix4();

    private final boolean LOW_RESOLUTION_SENSORS = true;
    private GDXL515SensorObject l515Model;

    public GDXPerceptionVisualizerUI()
    {
        ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, ROS2Tools.REA_NODE_NAME);
        RosMainNode ros1Node = new RosMainNode(NetworkParameters.getROSURI(), "PerceptionVisualizer");

        baseUI = new GDXImGuiBasedUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/libgdx/resources", "Perception Visualizer");
        mapsenseConfigurationUI = new ImGuiMapSenseConfigurationPanel(ros1Node, ros2Node);

        globalVisualizersUI = new ImGuiGDXGlobalVisualizersPanel();
        GDXROS1PlanarRegionsVisualizer mapsenseRegionsVisualizer = new GDXROS1PlanarRegionsVisualizer("MapSense Planar Regions",
                                                                                                      ros2Node,
                                                                                                      RosTools.MAPSENSE_REGIONS);
        globalVisualizersUI.addVisualizer(mapsenseRegionsVisualizer);
        globalVisualizersUI.addVisualizer(new GDXROS1PointCloudVisualizer("Head Ouster", RosTools.OUSTER_POINT_CLOUD));
        globalVisualizersUI.addVisualizer(new GDXROS1VideoVisualizer("L515 Color Video", RosTools.L515_VIDEO));
        globalVisualizersUI.addVisualizer(new GDXROS1VideoVisualizer("L515 Depth Video", RosTools.L515_DEPTH));
        globalVisualizersUI.addVisualizer(new GDXROS1VideoVisualizer("L515 Compressed Video", RosTools.L515_COMPRESSED_VIDEO));
        globalVisualizersUI.addVisualizer(new GDXROS2PointCloudVisualizer("MultiSense lidar scan", ros2Node, ROS2Tools.MULTISENSE_LIDAR_SCAN));
        globalVisualizersUI.addVisualizer(new GDXROS2PlanarRegionsVisualizer("Lidar REA planar regions", ros2Node, ROS2Tools.LIDAR_REA_REGIONS));
        globalVisualizersUI.addVisualizer(new GDXROS1OdometryVisualizer("SLAM Poses", RosTools.SLAM_POSE));
        globalVisualizersUI.addVisualizer(new GDXROS1OdometryVisualizer("Internal Sensor Pose", "/atlas/sensors/chest_l515/pose"));

        environmentBuilderUI = new GDXEnvironmentBuilderPanel();

//        simulatedDepthSensor = createSimulatedDepthSensor(ros1Node);
//        simulatedDepthSensor.setSensorFrameToWorldTransform(depthSensorTransform);
//        simulatedDepthSensor.setRenderPointCloudDirectly(true);
//        simulatedDepthSensor.setDebugCoordinateFrame(true);

        baseUI.getImGuiPanelManager().addPanel(globalVisualizersUI);
        baseUI.getImGuiPanelManager().addPanel(mapsenseRegionsVisualizer.getLoggingPanel());
//        baseUI.getImGuiDockingSetup().addWindow(simulatedDepthSensor.getWindowName(), simulatedDepthSensor::renderImGuiWindow);
        baseUI.getImGuiPanelManager().addPanel(mapsenseConfigurationUI.getWindowName(), mapsenseConfigurationUI::render);
        baseUI.getImGuiPanelManager().addPanel(environmentBuilderUI.getWindowName(), environmentBuilderUI::renderImGuiWindow);

        baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
        {
            @Override
            public void create()
            {
                baseUI.create();

                baseUI.get3DSceneManager().addRenderableProvider(globalVisualizersUI, GDXSceneLevel.VIRTUAL);

//                simulatedDepthSensor.create();
//                baseUI.getSceneManager().addRenderableProvider(simulatedDepthSensor, GDXSceneLevel.VIRTUAL);

                globalVisualizersUI.create();
                environmentBuilderUI.create(baseUI);
//                l515Model = new GDXL515SensorObject();
//                environmentBuilderUI.getModelInput().addInstance(l515Model);
                baseUI.get3DSceneManager().addRenderableProvider(environmentBuilderUI);

                ros1Node.execute();
            }

            @Override
            public void render()
            {
//                gdxSensorTransform.set(l515Model.getRealisticModelInstance().transform);
//                GDXTools.toEuclid(gdxSensorTransform, depthSensorTransform);
//                simulatedDepthSensor.render(baseUI.getSceneManager());
                globalVisualizersUI.update();

                baseUI.renderBeforeOnScreenUI();
                baseUI.renderEnd();
            }

            @Override
            public void dispose()
            {
//                simulatedDepthSensor.dispose();
                globalVisualizersUI.destroy();
                baseUI.dispose();
                ros2Node.destroy();
                ros1Node.shutdown();
            }
        });
    }

    public void regionsCallback(PlanarRegionsListMessage regions)
    {
        this.regionsUpdate = PlanarRegionMessageConverter.convertToPlanarRegionsList(regions);
    }

    public void setupPlanarRegionSLAM(ROS2Node ros2Node)
    {
        new IHMCROS2Callback<>(ros2Node, ROS2Tools.MAPSENSE_REGIONS, this::regionsCallback);
        new PeriodicPlanarRegionPublisher(ros2Node, ROS2Tools.REALSENSE_SLAM_REGIONS, 0.001f, () -> realsensePlanarRegionSLAM.update(regionsUpdate)).start();
    }

    private GDXHighLevelDepthSensorSimulator createSimulatedDepthSensor(RosNodeInterface ros1Node)
    {
        double publishRateHz = 5.0;
        double verticalFOV = 55.0;
        int imageWidth = 640;
        int imageHeight = 480;
        double fx = 500.0;
        double fy = 500.0;
        if (LOW_RESOLUTION_SENSORS)
        {
            imageWidth /= 2;
            imageHeight /= 2;
            fx /= 2;
            fy /= 2;
        }
        double minRange = 0.105;
        double maxRange = 5.0;
        CameraPinholeBrown depthCameraIntrinsics = new CameraPinholeBrown(fx, fy, 0, imageWidth / 2.0, imageHeight / 2.0, imageWidth, imageHeight);
        ROS2NodeInterface ros2Node = null;
        ROS2Topic<?> ros2Topic = null;
        ReferenceFrame sensorFrame = null;
        ROS2SyncedRobotModel syncedRobot = null;
        return new GDXHighLevelDepthSensorSimulator("L515",
                                                    ros1Node,
                                                    RosTools.MAPSENSE_DEPTH_IMAGE,
                                                    RosTools.MAPSENSE_DEPTH_CAMERA_INFO,
                                                    depthCameraIntrinsics,
                                                    RosTools.L515_VIDEO,
                                                    RosTools.L515_COLOR_CAMERA_INFO,
                                                    ros2Node,
                                                    ros2Topic,
                                                    null,
                                                    sensorFrame,
                                                    syncedRobot::getTimestamp,
                                                    verticalFOV,
                                                    imageWidth,
                                                    imageHeight,
                                                    minRange,
                                                    maxRange,
                                                    publishRateHz,
                                                    false);
    }

    public static void main(String[] args) throws URISyntaxException
    {
        new GDXPerceptionVisualizerUI();
    }
}

