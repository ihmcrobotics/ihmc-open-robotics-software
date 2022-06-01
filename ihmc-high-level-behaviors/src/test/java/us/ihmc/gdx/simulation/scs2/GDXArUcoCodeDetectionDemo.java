package us.ihmc.gdx.simulation.scs2;

import imgui.internal.ImGui;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.perception.GDXOpenCVArUcoMarkerDetectionUI;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.environment.GDXEnvironmentBuilder;
import us.ihmc.gdx.simulation.sensors.GDXHighLevelDepthSensorSimulator;
import us.ihmc.gdx.simulation.sensors.GDXSimulatedSensorFactory;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.perception.OpenCVArUcoMarker;
import us.ihmc.perception.OpenCVArUcoMarkerDetection;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.tools.io.WorkspaceFile;
import us.ihmc.tools.thread.Activator;

import java.util.ArrayList;

public class GDXArUcoCodeDetectionDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");
   private final Activator nativesLoadedActivator;
   private GDXEnvironmentBuilder environmentBuilder;
   private final GDXPose3DGizmo sensorPoseGizmo = new GDXPose3DGizmo();
   private GDXHighLevelDepthSensorSimulator cameraSensor;
   private BytedecoImage testRGB888ColorImage;
   private OpenCVArUcoMarkerDetection arUcoMarkerDetection;
   private GDXOpenCVArUcoMarkerDetectionUI arUcoMarkerDetectionUI;
   private OpenCVArUcoMarkerDetection testImageArUcoMarkerDetection;
   private GDXOpenCVArUcoMarkerDetectionUI testImageArUcoMarkerDetectionUI;

   public GDXArUcoCodeDetectionDemo()
   {
      nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();

      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            environmentBuilder = new GDXEnvironmentBuilder(baseUI.get3DSceneManager());
            environmentBuilder.create(baseUI);
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            baseUI.get3DSceneManager().addRenderableProvider(environmentBuilder::getRealRenderables, GDXSceneLevel.REAL_ENVIRONMENT);
            baseUI.get3DSceneManager().addRenderableProvider(environmentBuilder::getVirtualRenderables, GDXSceneLevel.VIRTUAL);
            environmentBuilder.loadEnvironment("DoorsForArUcoTesting.json");

            sensorPoseGizmo.create(baseUI.get3DSceneManager().getCamera3D());
            sensorPoseGizmo.setResizeAutomatically(true);
            baseUI.addImGui3DViewPickCalculator(sensorPoseGizmo::calculate3DViewPick);
            baseUI.addImGui3DViewInputProcessor(sensorPoseGizmo::process3DViewInput);
            baseUI.get3DSceneManager().addRenderableProvider(sensorPoseGizmo, GDXSceneLevel.VIRTUAL);
            sensorPoseGizmo.getTransformToParent().appendTranslation(0.0, 0.0, 1.0);
         }

         @Override
         public void render()
         {
            environmentBuilder.update();

            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  cameraSensor = GDXSimulatedSensorFactory.createBlackflyFisheyeImageOnlyNoComms(sensorPoseGizmo.getGizmoFrame());
                  cameraSensor.setSensorEnabled(true);
                  cameraSensor.setRenderColorVideoDirectly(true);
                  baseUI.getImGuiPanelManager().addPanel(cameraSensor);
                  baseUI.get3DSceneManager().addRenderableProvider(cameraSensor, GDXSceneLevel.VIRTUAL);

                  arUcoMarkerDetection = new OpenCVArUcoMarkerDetection();
                  arUcoMarkerDetection.create(cameraSensor.getLowLevelSimulator().getRGBA8888ColorImage(),
                                              cameraSensor.getDepthCameraIntrinsics(),
                                              cameraSensor.getSensorFrame());
                  arUcoMarkerDetectionUI = new GDXOpenCVArUcoMarkerDetectionUI("from Sensor");
                  ArrayList<OpenCVArUcoMarker> markersToTrack = new ArrayList<>();
                  markersToTrack.add(new OpenCVArUcoMarker(0, 0.2032));
                  markersToTrack.add(new OpenCVArUcoMarker(1, 0.2032));
                  arUcoMarkerDetectionUI.create(arUcoMarkerDetection, markersToTrack, sensorPoseGizmo.getGizmoFrame());
                  baseUI.getImGuiPanelManager().addPanel(arUcoMarkerDetectionUI.getMainPanel());
                  baseUI.get3DSceneManager().addRenderableProvider(arUcoMarkerDetectionUI::getRenderables, GDXSceneLevel.VIRTUAL);

                  loadTestImage();

                  testImageArUcoMarkerDetection = new OpenCVArUcoMarkerDetection();
                  testImageArUcoMarkerDetection.create(testRGB888ColorImage,
                                                       cameraSensor.getDepthCameraIntrinsics(),
                                                       cameraSensor.getSensorFrame());
                  testImageArUcoMarkerDetectionUI = new GDXOpenCVArUcoMarkerDetectionUI("Test");
                  testImageArUcoMarkerDetectionUI.create(testImageArUcoMarkerDetection, new ArrayList<>(), sensorPoseGizmo.getGizmoFrame());
                  ImGuiPanel testUIPanel = new ImGuiPanel("Test image detection", this::renderTestUIImGuiWidgets);
                  testUIPanel.addChild(testImageArUcoMarkerDetectionUI.getMarkerImagePanel().getVideoPanel());
                  baseUI.getImGuiPanelManager().addPanel(testUIPanel);


                  baseUI.getPerspectiveManager().reloadPerspective();
               }

               cameraSensor.render(baseUI.get3DSceneManager());

               arUcoMarkerDetection.update();
               arUcoMarkerDetectionUI.update();
               testImageArUcoMarkerDetection.update();
               testImageArUcoMarkerDetectionUI.update();
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void loadTestImage()
         {
            WorkspaceFile testImageFile = new WorkspaceFile(new WorkspaceDirectory("ihmc-open-robotics-software",
                                                                                   "ihmc-high-level-behaviors/src/test/resources"),
                                                            "testArUcoDetection.jpg");
            Mat readImage = opencv_imgcodecs.imread(testImageFile.getFilePath().toString());
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
      new GDXArUcoCodeDetectionDemo();
   }
}
