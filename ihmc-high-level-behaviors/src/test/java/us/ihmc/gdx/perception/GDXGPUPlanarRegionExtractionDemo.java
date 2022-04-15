package us.ihmc.gdx.perception;

import boofcv.struct.calib.CameraPinholeBrown;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.environment.GDXEnvironmentBuilder;
import us.ihmc.gdx.simulation.sensors.GDXHighLevelDepthSensorSimulator;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.affordances.GDXInteractableReferenceFrame;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.tools.thread.Activator;

public class GDXGPUPlanarRegionExtractionDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");
   private Activator nativesLoadedActivator;
   private GDXHighLevelDepthSensorSimulator l515;
   private GDXInteractableReferenceFrame robotInteractableReferenceFrame;
   private GDXPose3DGizmo l515PoseGizmo = new GDXPose3DGizmo();
   private GDXEnvironmentBuilder environmentBuilder;
   private GDXGPUPlanarRegionExtraction gpuPlanarRegionExtraction;

   public GDXGPUPlanarRegionExtractionDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();

            baseUI.create();

            environmentBuilder = new GDXEnvironmentBuilder(baseUI.get3DSceneManager());
            environmentBuilder.create(baseUI);
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            baseUI.get3DSceneManager().addRenderableProvider(environmentBuilder::getRealRenderables, GDXSceneLevel.REAL_ENVIRONMENT);
            baseUI.get3DSceneManager().addRenderableProvider(environmentBuilder::getVirtualRenderables, GDXSceneLevel.VIRTUAL);
            environmentBuilder.loadEnvironment("DemoPullDoor.json");

            robotInteractableReferenceFrame = new GDXInteractableReferenceFrame();
            robotInteractableReferenceFrame.create(ReferenceFrame.getWorldFrame(), 0.15, baseUI.get3DSceneManager().getCamera3D());
            robotInteractableReferenceFrame.getTransformToParent().getTranslation().add(2.2, 0.0, 1.0);
            baseUI.addImGui3DViewInputProcessor(robotInteractableReferenceFrame::process3DViewInput);
            baseUI.get3DSceneManager().addRenderableProvider(robotInteractableReferenceFrame::getVirtualRenderables, GDXSceneLevel.VIRTUAL);
            l515PoseGizmo = new GDXPose3DGizmo(robotInteractableReferenceFrame.getRepresentativeReferenceFrame());
            l515PoseGizmo.create(baseUI.get3DSceneManager().getCamera3D());
            l515PoseGizmo.setResizeAutomatically(false);
            baseUI.addImGui3DViewPickCalculator(l515PoseGizmo::calculate3DViewPick);
            baseUI.addImGui3DViewInputProcessor(l515PoseGizmo::process3DViewInput);
            baseUI.get3DSceneManager().addRenderableProvider(l515PoseGizmo, GDXSceneLevel.VIRTUAL);
            l515PoseGizmo.getTransformToParent().appendPitchRotation(Math.toRadians(60.0));
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  double publishRateHz = 5.0;
                  double verticalFOV = 55.0;
                  int imageWidth = 1024;
                  int imageHeight = 768;
                  double minRange = 0.105;
                  double maxRange = 5.0;
                  l515 = new GDXHighLevelDepthSensorSimulator("Stepping L515",
                                                              null,
                                                              null,
                                                              null,
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
                  l515.create();
                  l515.setSensorEnabled(true);
                  l515.setPublishPointCloudROS2(false);
                  l515.setRenderPointCloudDirectly(false);
                  l515.setPublishDepthImageROS1(false);
                  l515.setDebugCoordinateFrame(false);
                  l515.setRenderColorVideoDirectly(true);
                  l515.setRenderDepthVideoDirectly(true);
                  l515.setPublishColorImageROS1(false);
                  l515.setPublishColorImageROS2(false);
                  CameraPinholeBrown cameraIntrinsics = l515.getDepthCameraIntrinsics();
                  baseUI.get3DSceneManager().addRenderableProvider(l515, GDXSceneLevel.VIRTUAL);

                  gpuPlanarRegionExtraction = new GDXGPUPlanarRegionExtraction();
                  gpuPlanarRegionExtraction.create(imageWidth,
                                                   imageHeight,
                                                   l515.getLowLevelSimulator().getMetersDepthFloatBuffer(),
                                                   cameraIntrinsics.getFx(),
                                                   cameraIntrinsics.getFy(),
                                                   cameraIntrinsics.getCx(),
                                                   cameraIntrinsics.getCy());
                  gpuPlanarRegionExtraction.getEnabled().set(true);
                  baseUI.getImGuiPanelManager().addPanel(gpuPlanarRegionExtraction.getPanel());
                  baseUI.get3DSceneManager().addRenderableProvider(gpuPlanarRegionExtraction::getVirtualRenderables, GDXSceneLevel.VIRTUAL);

                  baseUI.getPerspectiveManager().reloadPerspective();
               }

               l515.render(baseUI.get3DSceneManager());
               gpuPlanarRegionExtraction.extractPlanarRegions(l515PoseGizmo.getGizmoFrame());
            }

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
