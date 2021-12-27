package us.ihmc.gdx.perception;

import boofcv.struct.calib.CameraPinholeBrown;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.environment.GDXEnvironmentBuilder;
import us.ihmc.gdx.simulation.sensors.GDXHighLevelDepthSensorSimulator;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;

public class GDXGPUPlanarRegionExtractionDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");

   private GDXHighLevelDepthSensorSimulator l515;
   private final GDXPose3DGizmo l515PoseGizmo = new GDXPose3DGizmo();
   private GDXEnvironmentBuilder environmentBuilder;
   private GDXGPUPlanarRegionExtraction gpuPlanarRegionExtraction;

   public GDXGPUPlanarRegionExtractionDemo()
   {
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
            environmentBuilder.loadEnvironment("DemoPullDoor.json");

            l515PoseGizmo.create(baseUI.get3DSceneManager().getCamera3D());
            l515PoseGizmo.setResizeAutomatically(false);
            baseUI.addImGui3DViewInputProcessor(l515PoseGizmo::process3DViewInput);
            baseUI.get3DSceneManager().addRenderableProvider(l515PoseGizmo, GDXSceneLevel.VIRTUAL);
            l515PoseGizmo.getTransformToParent().appendTranslation(0.0, 0.0, 0.5);
            l515PoseGizmo.getTransformToParent().appendPitchRotation(Math.PI / 6.0);

            double publishRateHz = 5.0;
            double verticalFOV = 55.0;
            int imageWidth = 640;
            int imageHeight = 480;
            double fx = 500.0;
            double fy = 500.0;
            double minRange = 0.105;
            double maxRange = 5.0;
            CameraPinholeBrown depthCameraIntrinsics = new CameraPinholeBrown(fx, fy, 0, imageWidth / 2.0, imageHeight / 2.0, imageWidth, imageHeight);
            l515 = new GDXHighLevelDepthSensorSimulator("Stepping L515",
                                                        null,
                                                        null,
                                                        null,
                                                        depthCameraIntrinsics,
                                                        null,
                                                        null,
                                                        null,
                                                        null,
                                                        null,
                                                        l515PoseGizmo.getGizmoFrame(),
                                                        () -> 0L,
                                                        verticalFOV,
                                                        imageWidth,
                                                        imageHeight,
                                                        minRange,
                                                        maxRange,
                                                        publishRateHz,
                                                        false);
            baseUI.getImGuiPanelManager().addPanel(l515);
            l515.setSensorEnabled(true);
            l515.setPublishPointCloudROS2(false);
            l515.setRenderPointCloudDirectly(false);
            l515.setPublishDepthImageROS1(false);
            l515.setDebugCoordinateFrame(false);
            l515.setRenderColorVideoDirectly(true);
            l515.setRenderDepthVideoDirectly(true);
            l515.setPublishColorImageROS1(false);
            l515.setPublishColorImageROS2(false);
            l515.create();
            baseUI.get3DSceneManager().addRenderableProvider(l515, GDXSceneLevel.VIRTUAL);

            gpuPlanarRegionExtraction = new GDXGPUPlanarRegionExtraction();
            gpuPlanarRegionExtraction.create(imageWidth, imageHeight, l515.getLowLevelSimulator().getEyeDepthMetersByteBuffer());
            baseUI.getImGuiPanelManager().addPanel(gpuPlanarRegionExtraction.getPanel());
         }

         @Override
         public void render()
         {
            l515.render(baseUI.get3DSceneManager());
            gpuPlanarRegionExtraction.blurDepthAndRender(l515.getLowLevelSimulator().getEyeDepthMetersByteBuffer());

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
            environmentBuilder.destroy();
            l515.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new GDXGPUPlanarRegionExtractionDemo();
   }
}
