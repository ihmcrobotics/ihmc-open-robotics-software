package us.ihmc.rdx.simulation.scs2;

import imgui.internal.ImGui;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.opencv.OpenCVArUcoMarker;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerDetector;
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

public class RDXArUcoMarkerDetectionDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private RDXEnvironmentBuilder environmentBuilder;
   private final RDXPose3DGizmo sensorPoseGizmo = new RDXPose3DGizmo();
   private RDXHighLevelDepthSensorSimulator cameraSensor;
   private BytedecoImage testRGB888ColorImage;
   private OpenCVArUcoMarkerDetector arUcoMarkerDetector;
   private RDXOpenCVArUcoMarkerDetectionUI arUcoMarkerDetectionUI;
   private OpenCVArUcoMarkerDetector testImageArUcoMarkerDetector;
   private RDXOpenCVArUcoMarkerDetectionUI testImageArUcoMarkerDetectionUI;

   public RDXArUcoMarkerDetectionDemo()
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

            arUcoMarkerDetector = new OpenCVArUcoMarkerDetector();
            arUcoMarkerDetector.setSourceImageForDetection(cameraSensor.getLowLevelSimulator().getRGBA8888ColorImage());
            arUcoMarkerDetector.setCameraInstrinsics(cameraSensor.getDepthCameraIntrinsics());
            arUcoMarkerDetectionUI = new RDXOpenCVArUcoMarkerDetectionUI(" from Sensor");
            ArrayList<OpenCVArUcoMarker> markersToTrack = new ArrayList<>();
            markersToTrack.add(new OpenCVArUcoMarker(0, 0.2032));
            markersToTrack.add(new OpenCVArUcoMarker(1, 0.2032));
            arUcoMarkerDetectionUI.create(arUcoMarkerDetector.getDetectorParameters());
            arUcoMarkerDetectionUI.setupForRenderingDetectedPosesIn3D(markersToTrack, sensorPoseGizmo.getGizmoFrame());
            baseUI.getImGuiPanelManager().addPanel(arUcoMarkerDetectionUI.getMainPanel());
            baseUI.getPrimaryScene().addRenderableProvider(arUcoMarkerDetectionUI::getRenderables, RDXSceneLevel.VIRTUAL);

            loadTestImage();

            testImageArUcoMarkerDetector = new OpenCVArUcoMarkerDetector();
            arUcoMarkerDetector.getDetectorParameters().markerBorderBits(2);
            testImageArUcoMarkerDetector.setSourceImageForDetection(testRGB888ColorImage);
            testImageArUcoMarkerDetector.setCameraInstrinsics(cameraSensor.getDepthCameraIntrinsics());
            testImageArUcoMarkerDetectionUI = new RDXOpenCVArUcoMarkerDetectionUI(" Test");
            testImageArUcoMarkerDetectionUI.create(testImageArUcoMarkerDetector.getDetectorParameters());
            testImageArUcoMarkerDetectionUI.setupForRenderingDetectedPosesIn3D(new ArrayList<>(), sensorPoseGizmo.getGizmoFrame());
            RDXPanel testUIPanel = new RDXPanel("Test image detection", this::renderTestUIImGuiWidgets);
            testUIPanel.addChild(testImageArUcoMarkerDetectionUI.getMarkerImagePanel());
            baseUI.getImGuiPanelManager().addPanel(testUIPanel);
         }

         @Override
         public void render()
         {
            environmentBuilder.update();

            cameraSensor.render(baseUI.getPrimaryScene());

            arUcoMarkerDetector.update();
            arUcoMarkerDetectionUI.copyOutputData(arUcoMarkerDetector);
            arUcoMarkerDetectionUI.update();
            testImageArUcoMarkerDetector.update();
            testImageArUcoMarkerDetectionUI.copyOutputData(testImageArUcoMarkerDetector);
            testImageArUcoMarkerDetectionUI.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void loadTestImage()
         {
            WorkspaceFile testImageFile = new WorkspaceFile(new WorkspaceDirectory("ihmc-open-robotics-software",
                                                                                   "ihmc-high-level-behaviors/src/test/resources"), "testArUcoDetection.jpg");
            Mat readImage = opencv_imgcodecs.imread(testImageFile.getFilesystemFile().toString());
            testRGB888ColorImage = new BytedecoImage(readImage);
         }

         private void renderTestUIImGuiWidgets()
         {
            if (ImGui.button("Reload test image"))
            {
               loadTestImage();
            }
            testImageArUcoMarkerDetectionUI.renderImGuiWidgets();
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
      new RDXArUcoMarkerDetectionDemo();
   }
}
