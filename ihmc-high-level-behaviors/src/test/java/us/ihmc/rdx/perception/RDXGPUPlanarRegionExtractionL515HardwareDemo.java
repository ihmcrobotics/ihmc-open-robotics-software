package us.ihmc.rdx.perception;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXInteractableReferenceFrame;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.MutableBytePointer;
import us.ihmc.perception.realsense.BytedecoRealsense;
import us.ihmc.perception.realsense.RealSenseHardwareManager;
import us.ihmc.tools.thread.Activator;

public class RDXGPUPlanarRegionExtractionL515HardwareDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI("ihmc-open-robotics-software",
                                                  "ihmc-high-level-behaviors/src/test/resources");
   private Activator nativesLoadedActivator;
   private RDXInteractableReferenceFrame robotInteractableReferenceFrame;
   private RealSenseHardwareManager realSenseHardwareManager;
   private BytedecoRealsense l515;
   private Mat depthU16C1Image;
   private BytedecoImage depth32FC1Image;
   private RDXPose3DGizmo l515PoseGizmo = new RDXPose3DGizmo();
   private RDXGPUPlanarRegionExtractionUI gpuPlanarRegionExtraction;

   public RDXGPUPlanarRegionExtractionL515HardwareDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();

            baseUI.create();

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
                  realSenseHardwareManager = new RealSenseHardwareManager();
//                  l515 = realSenseHardwareManager.createFullFeaturedL515("F1120418");
                  l515 = realSenseHardwareManager.createFullFeaturedL515("F1121365");
                  l515.initialize();
               }

               if (l515.readFrameData())
               {
                  l515.updateDataBytePointers();

                  if (gpuPlanarRegionExtraction == null)
                  {
                     MutableBytePointer depthFrameData = l515.getDepthFrameData();
                     depthU16C1Image = new Mat(l515.getDepthHeight(), l515.getDepthWidth(), opencv_core.CV_16UC1, depthFrameData);

                     depth32FC1Image = new BytedecoImage(l515.getDepthWidth(), l515.getDepthHeight(), opencv_core.CV_32FC1);

                     gpuPlanarRegionExtraction = new RDXGPUPlanarRegionExtractionUI();
                     gpuPlanarRegionExtraction.create(l515.getDepthWidth(),
                                                      l515.getDepthHeight(),
                                                      depth32FC1Image.getBackingDirectByteBuffer(),
                                                      l515.getDepthIntrinsicParameters().fx(),
                                                      l515.getDepthIntrinsicParameters().fy(),
                                                      l515.getDepthIntrinsicParameters().ppx(),
                                                      l515.getDepthIntrinsicParameters().ppy(),
                                                      l515PoseGizmo.getGizmoFrame());
                     gpuPlanarRegionExtraction.getEnabled().set(true);
                     baseUI.getImGuiPanelManager().addPanel(gpuPlanarRegionExtraction.getPanel());
                     baseUI.getPrimaryScene().addRenderableProvider(gpuPlanarRegionExtraction::getVirtualRenderables, RDXSceneLevel.VIRTUAL);

                     baseUI.getLayoutManager().reloadLayout();
                  }

                  depthU16C1Image.convertTo(depth32FC1Image.getBytedecoOpenCVMat(), opencv_core.CV_32FC1, l515.getDepthToMeterConversion(), 0.0);
                  gpuPlanarRegionExtraction.extractPlanarRegions();
               }
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXGPUPlanarRegionExtractionL515HardwareDemo();
   }
}
