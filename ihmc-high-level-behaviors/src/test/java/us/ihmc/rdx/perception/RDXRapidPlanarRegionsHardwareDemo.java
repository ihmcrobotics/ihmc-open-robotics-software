package us.ihmc.rdx.perception;

import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.MutableBytePointer;
import us.ihmc.perception.realsense.RealsenseDevice;
import us.ihmc.perception.realsense.RealsenseDeviceManager;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXInteractableReferenceFrame;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;

public class RDXRapidPlanarRegionsHardwareDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private RDXInteractableReferenceFrame robotInteractableReferenceFrame;
   private RealsenseDeviceManager realsenseDeviceManager;
   private RealsenseDevice l515;
   private Mat depthU16C1Image;
   private BytedecoImage bytedecoDepthImage;
   private RDXPose3DGizmo l515PoseGizmo = new RDXPose3DGizmo();
   private RDXRapidRegionsUI rapidRegionsUI = new RDXRapidRegionsUI();
   private RapidPlanarRegionsExtractor rapidRegionsExtractor;
   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;

   public RDXRapidPlanarRegionsHardwareDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
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

            realsenseDeviceManager = new RealsenseDeviceManager();
            //                  l515 = realSenseHardwareManager.createFullFeaturedL515("F1120418");
            l515 = realsenseDeviceManager.createFullFeaturedL515("F1121365");
            l515.initialize();
         }

         @Override
         public void render()
         {
            if (l515.readFrameData())
            {
               l515.updateDataBytePointers();

               if (openCLManager == null)
               {
                  openCLManager = new OpenCLManager();
                  openCLProgram = openCLManager.loadProgram("RapidRegionsExtractor");

                  realsenseDeviceManager = new RealsenseDeviceManager();
                  l515 = realsenseDeviceManager.createFullFeaturedL515("F1121365");
                  l515.initialize();
               }

               if (l515.readFrameData())
               {
                  l515.updateDataBytePointers();

                  if (rapidRegionsUI == null)
                  {
                     MutableBytePointer depthFrameData = l515.getDepthFrameData();
                     depthU16C1Image = new Mat(l515.getDepthHeight(), l515.getDepthWidth(), opencv_core.CV_16UC1, depthFrameData);

                     bytedecoDepthImage = new BytedecoImage(l515.getDepthWidth(), l515.getDepthHeight(), opencv_core.CV_16UC1);

                     rapidRegionsExtractor = new RapidPlanarRegionsExtractor(openCLManager,
                                                                             openCLProgram,
                                                                             l515.getDepthWidth(),
                                                                             l515.getDepthHeight(),
                                                                             l515.getDepthIntrinsicParameters().fx(),
                                                                             l515.getDepthIntrinsicParameters().fy(),
                                                                             l515.getDepthIntrinsicParameters().ppx(),
                                                                             l515.getDepthIntrinsicParameters().ppy());

                     rapidRegionsUI.getEnabled().set(true);
                     baseUI.getImGuiPanelManager().addPanel(rapidRegionsUI.getPanel());
                     baseUI.getPrimaryScene().addRenderableProvider(rapidRegionsUI::getRenderables, RDXSceneLevel.VIRTUAL);

                     baseUI.getLayoutManager().reloadLayout();
                  }

                  if (rapidRegionsUI.getEnabled().get())
                  {
                     depthU16C1Image.convertTo(bytedecoDepthImage.getBytedecoOpenCVMat(), opencv_core.CV_16UC1, 1, 0);

                     // Get the planar regions from the planar region extractor
                     FramePlanarRegionsList frameRegions = new FramePlanarRegionsList();
                     rapidRegionsExtractor.update(bytedecoDepthImage, l515PoseGizmo.getGizmoFrame(), frameRegions);
                     rapidRegionsExtractor.setProcessing(false);

                     if (rapidRegionsExtractor.isModified())
                     {
                        rapidRegionsUI.render3DGraphics(frameRegions);
                        rapidRegionsExtractor.setProcessing(false);
                     }
                  }
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
      new RDXRapidPlanarRegionsHardwareDemo();
   }
}
