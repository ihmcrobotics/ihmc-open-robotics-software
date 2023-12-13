package us.ihmc.rdx.simulation.scs2;

import imgui.internal.ImGui;
import javafx.application.Application;
import javafx.stage.Stage;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.opencv.OpenCVArUcoMarker;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerDetection;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.perception.RDXOpenCVArUcoMarkerDetectionUI;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;
import us.ihmc.rdx.simulation.sensors.RDXSimulatedSensorFactory;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.tools.io.WorkspaceFile;

import java.util.ArrayList;

public class RDXYOLOv8WithONNXRuntimeDemo
    {
        private final RDXBaseUI baseUI = new RDXBaseUI();
        private RDXEnvironmentBuilder environmentBuilder;
        private final RDXPose3DGizmo sensorPoseGizmo = new RDXPose3DGizmo();
        private RDXHighLevelDepthSensorSimulator cameraSensor;
//        private OpenCVArUcoMarkerDetection arUcoMarkerDetection;
//        private RDXOpenCVArUcoMarkerDetectionUI arUcoMarkerDetectionUI;


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

//                    cameraSensor = RDXSimulatedSensorFactory.createBlackflyFisheyeImageOnlyNoComms(sensorPoseGizmo.getGizmoFrame());
                    cameraSensor = RDXSimulatedSensorFactory.cre
                    cameraSensor.setSensorEnabled(true);
                    cameraSensor.setRenderColorVideoDirectly(true);
                    baseUI.getImGuiPanelManager().addPanel(cameraSensor);
                    baseUI.getPrimaryScene().addRenderableProvider(cameraSensor::getRenderables);

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
