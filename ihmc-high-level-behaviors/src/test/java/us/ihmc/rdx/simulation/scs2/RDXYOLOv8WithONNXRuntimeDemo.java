package us.ihmc.rdx.simulation.scs2;

import imgui.internal.ImGui;
import javafx.application.Application;
import javafx.stage.Stage;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.opencv.OpenCVArUcoMarker;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerDetection;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.NativeMemoryTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.perception.RDXOpenCVArUcoMarkerDetectionUI;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;
import us.ihmc.rdx.simulation.sensors.RDXSimulatedSensorFactory;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizerPanel;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2CenterposeVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.tools.io.WorkspaceFile;
import us.ihmc.tools.string.StringTools;

import java.util.ArrayList;

public class RDXYOLOv8WithONNXRuntimeDemo
    {
        private final RDXBaseUI baseUI = new RDXBaseUI();
        private RDXEnvironmentBuilder environmentBuilder;
        private final RDXPose3DGizmo sensorPoseGizmo = new RDXPose3DGizmo();
        private RDXHighLevelDepthSensorSimulator cameraSensor;
//        private OpenCVArUcoMarkerDetection arUcoMarkerDetection;
//        private RDXOpenCVArUcoMarkerDetectionUI arUcoMarkerDetectionUI;

        private final ImageMessage imageMessage = new ImageMessage();
        private final SampleInfo sampleInfo = new SampleInfo();


   public RDXYOLOv8WithONNXRuntimeDemo()
        {
            baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
            {
                @Override
                public void create()
                {
                    baseUI.create();

                    environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
                    environmentBuilder.create();
                    baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
                    environmentBuilder.loadEnvironment("DoorsForArUcoTesting.json");

                    sensorPoseGizmo.create(baseUI.getPrimary3DPanel());
                    sensorPoseGizmo.setResizeAutomatically(true);
                    baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(sensorPoseGizmo::calculate3DViewPick);
                    baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(sensorPoseGizmo::process3DViewInput);
                    baseUI.getPrimaryScene().addRenderableProvider(sensorPoseGizmo, RDXSceneLevel.VIRTUAL);
                    sensorPoseGizmo.getTransformToParent().appendTranslation(0.0, 0.0, 1.0);

                    cameraSensor = RDXSimulatedSensorFactory.createBlackflyFisheyeImageOnlyNoComms(sensorPoseGizmo.getGizmoFrame());
                    cameraSensor.setSensorEnabled(true);
                    cameraSensor.setRenderColorVideoDirectly(true);
                    baseUI.getImGuiPanelManager().addPanel(cameraSensor);
                    baseUI.getPrimaryScene().addRenderableProvider(cameraSensor::getRenderables);

                    String titleBeforeAdditions = "ZED 2 Color Left";
                    DomainFactory.PubSubImplementation pubSubImplementation = DomainFactory.PubSubImplementation.FAST_RTPS;
                    ROS2Topic<ImageMessage> topic = PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT);
                    RealtimeROS2Node realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(pubSubImplementation, StringTools.titleToSnakeCase(titleBeforeAdditions));
                    ROS2Tools.createCallbackSubscription(realtimeROS2Node, topic, ROS2QosProfile.BEST_EFFORT(), this::getLeftImage);

//                    RDXROS2ImageMessageVisualizer zedLeftColorVisualizer = new RDXROS2ImageMessageVisualizer("ZED 2 Color Left",
//                                                                                                                DomainFactory.PubSubImplementation.FAST_RTPS,
//                                                                                                                PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT));


//                    arUcoMarkerDetection = new OpenCVArUcoMarkerDetection();
//                    arUcoMarkerDetection.create(cameraSensor.getSensorFrame());
//                    arUcoMarkerDetection.setSourceImageForDetection(cameraSensor.getLowLevelSimulator().getRGBA8888ColorImage());
//                    arUcoMarkerDetection.setCameraInstrinsics(cameraSensor.getDepthCameraIntrinsics());
//                    arUcoMarkerDetectionUI = new RDXOpenCVArUcoMarkerDetectionUI(" from Sensor");
//                    ArrayList<OpenCVArUcoMarker> markersToTrack = new ArrayList<>();
//                    markersToTrack.add(new OpenCVArUcoMarker(0, 0.2032));
//                    markersToTrack.add(new OpenCVArUcoMarker(1, 0.2032));
//                    arUcoMarkerDetectionUI.create(arUcoMarkerDetection);
//                    arUcoMarkerDetectionUI.setupForRenderingDetectedPosesIn3D(markersToTrack, sensorPoseGizmo.getGizmoFrame());
//                    baseUI.getImGuiPanelManager().addPanel(arUcoMarkerDetectionUI.getMainPanel());
//                    baseUI.getPrimaryScene().addRenderableProvider(arUcoMarkerDetectionUI::getRenderables, RDXSceneLevel.VIRTUAL);

                }

                private void getLeftImage(Subscriber<ImageMessage> subscriber)
                {
                        imageMessage.getData().resetQuick();
                        subscriber.takeNextData(imageMessage, sampleInfo);
                }

                @Override
                public void render()
                {
                    environmentBuilder.update();

                    cameraSensor.render(baseUI.getPrimaryScene());

//                    arUcoMarkerDetection.update();
//                    arUcoMarkerDetectionUI.update();
//                    testImageArUcoMarkerDetection.update();
//                    testImageArUcoMarkerDetectionUI.update();

                    baseUI.renderBeforeOnScreenUI();
                    baseUI.renderEnd();
                }


                @Override
                public void dispose()
                {
                    baseUI.dispose();
                    environmentBuilder.destroy();
                    cameraSensor.dispose();
                }
            });
        }

        public static void main(String[] args)
        {
            new RDXYOLOv8WithONNXRuntimeDemo();
        }
    }
