package us.ihmc.gdx.perception;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.affordances.GDXInteractableReferenceFrame;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.MutableBytePointer;
import us.ihmc.perception.realsense.BytedecoRealsense;
import us.ihmc.perception.realsense.RealSenseHardwareManager;
import us.ihmc.tools.thread.Activator;

public class GDXGPUPlanarRegionExtractionL515HardwareDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");
   private Activator nativesLoadedActivator;
   private GDXInteractableReferenceFrame robotInteractableReferenceFrame;
   private RealSenseHardwareManager realSenseHardwareManager;
   private BytedecoRealsense l515;
   private Mat depthU16C1Image;
   private BytedecoImage depth32FC1Image;
   private GDXPose3DGizmo l515PoseGizmo = new GDXPose3DGizmo();
   private GDXGPUPlanarRegionExtractionUI gpuPlanarRegionExtraction;

   public GDXGPUPlanarRegionExtractionL515HardwareDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();

            baseUI.create();

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

                     gpuPlanarRegionExtraction = new GDXGPUPlanarRegionExtractionUI();
                     gpuPlanarRegionExtraction.create(l515.getDepthWidth(),
                                                      l515.getDepthHeight(),
                                                      depth32FC1Image.getBackingDirectByteBuffer(),
                                                      l515.getDepthCameraIntrinsics().fx,
                                                      l515.getDepthCameraIntrinsics().fy,
                                                      l515.getDepthCameraIntrinsics().cx,
                                                      l515.getDepthCameraIntrinsics().cy);
                     gpuPlanarRegionExtraction.getEnabled().set(true);
                     baseUI.getImGuiPanelManager().addPanel(gpuPlanarRegionExtraction.getPanel());
                     baseUI.getPrimaryScene().addRenderableProvider(gpuPlanarRegionExtraction::getVirtualRenderables, GDXSceneLevel.VIRTUAL);

                     baseUI.getPerspectiveManager().reloadPerspective();
                  }

                  depthU16C1Image.convertTo(depth32FC1Image.getBytedecoOpenCVMat(), opencv_core.CV_32FC1, l515.getDepthToMeterConversion(), 0.0);
                  gpuPlanarRegionExtraction.extractPlanarRegions(l515PoseGizmo.getGizmoFrame());
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
      new GDXGPUPlanarRegionExtractionL515HardwareDemo();
   }
}
