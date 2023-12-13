package us.ihmc.rdx.perception;

import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;
import us.ihmc.rdx.simulation.sensors.RDXSimulatedSensorFactory;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXInteractableReferenceFrame;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;

public class RDXPlanarRegionsSimulationDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private RDXHighLevelDepthSensorSimulator steppingL515Simulator;
   private OpenCLManager openCLManager;
   private RDXInteractableReferenceFrame robotInteractableReferenceFrame;
   private RDXPose3DGizmo l515PoseGizmo = new RDXPose3DGizmo();
   private RDXEnvironmentBuilder environmentBuilder;
   private RDXRapidRegionsUI rapidRegionsUI = new RDXRapidRegionsUI();
   private RapidPlanarRegionsExtractor rapidPlanarRegionsExtractor;
   private BytedecoImage bytedecoDepthImage;

   private boolean initialized = false;

   public RDXPlanarRegionsSimulationDemo()
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
               if (!initialized)
               {
                  openCLManager = new OpenCLManager();

                  steppingL515Simulator = RDXSimulatedSensorFactory.createRealsenseL515(l515PoseGizmo.getGizmoFrame(), () -> 0L);
                  baseUI.getImGuiPanelManager().addPanel(steppingL515Simulator);
                  steppingL515Simulator.setSensorEnabled(true);
                  steppingL515Simulator.setPublishPointCloudROS2(false);
                  steppingL515Simulator.setRenderPointCloudDirectly(false);
                  steppingL515Simulator.setPublishDepthImageROS1(false);
                  steppingL515Simulator.setDebugCoordinateFrame(false);
                  steppingL515Simulator.setRenderColorVideoDirectly(true);
                  steppingL515Simulator.setRenderDepthVideoDirectly(true);
                  steppingL515Simulator.setPublishColorImageROS1(false);
                  steppingL515Simulator.setPublishColorImageROS2(false);
                  baseUI.getPrimaryScene().addRenderableProvider(steppingL515Simulator::getRenderables);

                  bytedecoDepthImage = new BytedecoImage(steppingL515Simulator.getLowLevelSimulator().getImageWidth(),
                                                         steppingL515Simulator.getLowLevelSimulator().getImageHeight(),
                                                         opencv_core.CV_16UC1);
                  bytedecoDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
                  rapidPlanarRegionsExtractor = new RapidPlanarRegionsExtractor(openCLManager, steppingL515Simulator.getCopyOfCameraParameters());

                  rapidRegionsUI.create(rapidPlanarRegionsExtractor);
                  baseUI.getImGuiPanelManager().addPanel(rapidRegionsUI.getPanel());
                  baseUI.getPrimaryScene().addRenderableProvider(rapidRegionsUI, RDXSceneLevel.VIRTUAL);

                  baseUI.getLayoutManager().reloadLayout();

                  initialized = true;
               }

               steppingL515Simulator.render(baseUI.getPrimaryScene());

               if (rapidRegionsUI.getEnabled().get())
               {
                  OpenCVTools.convertFloatToShort(steppingL515Simulator.getLowLevelSimulator().getMetersDepthOpenCVMat(),
                                                  bytedecoDepthImage.getBytedecoOpenCVMat(),
                                                  1000.0,
                                                  0.0);

                  // Get the planar regions from the planar region extractor
                  FramePlanarRegionsList frameRegions = new FramePlanarRegionsList();
                  rapidPlanarRegionsExtractor.update(bytedecoDepthImage, steppingL515Simulator.getSensorFrame(), frameRegions);
                  rapidPlanarRegionsExtractor.setProcessing(false);

                  if (rapidPlanarRegionsExtractor.isModified())
                  {
                     rapidRegionsUI.render3DGraphics(frameRegions);
                     rapidPlanarRegionsExtractor.setProcessing(false);
                  }
               }


            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
            environmentBuilder.destroy();
            steppingL515Simulator.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXPlanarRegionsSimulationDemo();
   }
}
