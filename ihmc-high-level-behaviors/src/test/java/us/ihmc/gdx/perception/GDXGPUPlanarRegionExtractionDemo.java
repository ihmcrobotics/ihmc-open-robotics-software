package us.ihmc.gdx.perception;

import boofcv.struct.calib.CameraPinholeBrown;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.environment.GDXEnvironmentBuilder;
import us.ihmc.gdx.simulation.sensors.GDXHighLevelDepthSensorSimulator;
import us.ihmc.gdx.simulation.sensors.GDXSimulatedSensorFactory;
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
   private GDXGPUPlanarRegionExtractionUI gpuPlanarRegionExtraction;

   public GDXGPUPlanarRegionExtractionDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();

            baseUI.create();

            environmentBuilder = new GDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
            environmentBuilder.create();
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            baseUI.getPrimaryScene().addRenderableProvider(environmentBuilder::getRealRenderables, GDXSceneLevel.REAL_ENVIRONMENT);
            baseUI.getPrimaryScene().addRenderableProvider(environmentBuilder::getVirtualRenderables, GDXSceneLevel.VIRTUAL);
            environmentBuilder.loadEnvironment("DemoPullDoor.json");

            robotInteractableReferenceFrame = new GDXInteractableReferenceFrame();
            robotInteractableReferenceFrame.create(ReferenceFrame.getWorldFrame(), 0.15, baseUI.getPrimary3DPanel().getCamera3D());
            robotInteractableReferenceFrame.getTransformToParent().getTranslation().add(2.2, 0.0, 1.0);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(robotInteractableReferenceFrame::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(robotInteractableReferenceFrame::getVirtualRenderables, GDXSceneLevel.VIRTUAL);
            l515PoseGizmo = new GDXPose3DGizmo(robotInteractableReferenceFrame.getRepresentativeReferenceFrame());
            l515PoseGizmo.create(baseUI.getPrimary3DPanel().getCamera3D());
            l515PoseGizmo.setResizeAutomatically(false);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(l515PoseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(l515PoseGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(l515PoseGizmo, GDXSceneLevel.VIRTUAL);
            l515PoseGizmo.getTransformToParent().appendPitchRotation(Math.toRadians(60.0));
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  l515 = GDXSimulatedSensorFactory.createRealsenseL515(l515PoseGizmo.getGizmoFrame(), () -> 0L);
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
                  CameraPinholeBrown cameraIntrinsics = l515.getDepthCameraIntrinsics();
                  baseUI.getPrimaryScene().addRenderableProvider(l515, GDXSceneLevel.VIRTUAL);

                  gpuPlanarRegionExtraction = new GDXGPUPlanarRegionExtractionUI();
                  gpuPlanarRegionExtraction.create(l515.getLowLevelSimulator().getImageWidth(),
                                                   l515.getLowLevelSimulator().getImageHeight(),
                                                   l515.getLowLevelSimulator().getMetersDepthFloatBuffer(),
                                                   cameraIntrinsics.getFx(),
                                                   cameraIntrinsics.getFy(),
                                                   cameraIntrinsics.getCx(),
                                                   cameraIntrinsics.getCy());
                  gpuPlanarRegionExtraction.getEnabled().set(true);
                  baseUI.getImGuiPanelManager().addPanel(gpuPlanarRegionExtraction.getPanel());
                  baseUI.getPrimaryScene().addRenderableProvider(gpuPlanarRegionExtraction::getVirtualRenderables, GDXSceneLevel.VIRTUAL);

                  baseUI.getPerspectiveManager().reloadPerspective();
               }

               l515.render(baseUI.getPrimaryScene());
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
