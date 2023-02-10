package us.ihmc.rdx.perception;

import boofcv.struct.calib.CameraPinholeBrown;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;
import us.ihmc.rdx.simulation.sensors.RDXSimulatedSensorFactory;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXInteractableReferenceFrame;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.tools.thread.Activator;

public class RDXGPUPlanarRegionExtractionDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI("ihmc-open-robotics-software",
                                                  "ihmc-high-level-behaviors/src/test/resources");
   private Activator nativesLoadedActivator;
   private RDXHighLevelDepthSensorSimulator l515;
   private RDXInteractableReferenceFrame robotInteractableReferenceFrame;
   private RDXPose3DGizmo l515PoseGizmo = new RDXPose3DGizmo();
   private RDXEnvironmentBuilder environmentBuilder;
   private RDXGPUPlanarRegionExtractionUI gpuPlanarRegionExtraction;

   public RDXGPUPlanarRegionExtractionDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();

            baseUI.create();

            environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
            environmentBuilder.create();
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            environmentBuilder.loadEnvironment("DemoPullDoor.json");

            robotInteractableReferenceFrame = new RDXInteractableReferenceFrame();
            robotInteractableReferenceFrame.create(ReferenceFrame.getWorldFrame(), 0.15, baseUI.getPrimary3DPanel());
            robotInteractableReferenceFrame.getTransformToParent().getTranslation().add(2.2, 0.0, 1.0);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(robotInteractableReferenceFrame::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(robotInteractableReferenceFrame::getVirtualRenderables, RDXSceneLevel.VIRTUAL);
            l515PoseGizmo = new RDXPose3DGizmo(robotInteractableReferenceFrame.getRepresentativeReferenceFrame());
            l515PoseGizmo.create(baseUI.getPrimary3DPanel());
            l515PoseGizmo.setResizeAutomatically(false);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(l515PoseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(l515PoseGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(l515PoseGizmo, RDXSceneLevel.VIRTUAL);
            l515PoseGizmo.getTransformToParent().appendPitchRotation(Math.toRadians(60.0));
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  l515 = RDXSimulatedSensorFactory.createRealsenseL515(l515PoseGizmo.getGizmoFrame(), () -> 0L);
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
                  baseUI.getPrimaryScene().addRenderableProvider(l515::getRenderables);

                  gpuPlanarRegionExtraction = new RDXGPUPlanarRegionExtractionUI();
                  gpuPlanarRegionExtraction.create(l515.getLowLevelSimulator().getImageWidth(),
                                                   l515.getLowLevelSimulator().getImageHeight(),
                                                   l515.getLowLevelSimulator().getMetersDepthFloatBuffer(),
                                                   cameraIntrinsics.getFx(),
                                                   cameraIntrinsics.getFy(),
                                                   cameraIntrinsics.getCx(),
                                                   cameraIntrinsics.getCy(),
                                                   l515PoseGizmo.getGizmoFrame());
                  gpuPlanarRegionExtraction.getEnabled().set(true);
                  baseUI.getImGuiPanelManager().addPanel(gpuPlanarRegionExtraction.getPanel());
                  baseUI.getPrimaryScene().addRenderableProvider(gpuPlanarRegionExtraction::getVirtualRenderables, RDXSceneLevel.VIRTUAL);

                  baseUI.getLayoutManager().reloadLayout();
               }

               l515.render(baseUI.getPrimaryScene());
               gpuPlanarRegionExtraction.extractPlanarRegions();
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
      new RDXGPUPlanarRegionExtractionDemo();
   }
}
