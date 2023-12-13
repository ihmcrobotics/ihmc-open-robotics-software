package us.ihmc.rdx.logging;

import boofcv.struct.calib.CameraPinholeBrown;
import imgui.ImGui;
import org.bytedeco.ffmpeg.global.avutil;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXInteractableReferenceFrame;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;

import java.nio.ByteOrder;

public class RDXFFMPEGL515DepthLoggingDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final boolean lossless = false;
   private final int framerate = 15;
   private final int bitrate = 1450000;
   private final FFMPEGLoggerDemoHelper ffmpegLoggerDemoHelper = new FFMPEGLoggerDemoHelper("FFMPEGL515DepthLoggingDemo.webm",
                                                                                            avutil.AV_PIX_FMT_RGBA,
                                                                                            avutil.AV_PIX_FMT_YUV420P,
                                                                                            lossless,
                                                                                            framerate,
                                                                                            bitrate);
   private RDXHighLevelDepthSensorSimulator l515;
   private RDXInteractableReferenceFrame robotInteractableReferenceFrame;
   private RDXPose3DGizmo l515PoseGizmo = new RDXPose3DGizmo();
   private RDXEnvironmentBuilder environmentBuilder;
   private int imageWidth;
   private int imageHeight;
   private BytedecoImage normalizedDepthImage;
   private BytedecoImage rgbaDepthImage;

   public RDXFFMPEGL515DepthLoggingDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            RDXPanel panel = new RDXPanel("Diagnostics", this::renderImGuiWidgets);
            baseUI.getImGuiPanelManager().addPanel(panel);

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

            double publishRateHz = 5.0;
            double verticalFOV = 55.0;
            imageWidth = 1024;
            imageHeight = 768;
            double minRange = 0.105;
            double maxRange = 5.0;
            l515 = new RDXHighLevelDepthSensorSimulator("Stepping L515",
                                                        l515PoseGizmo.getGizmoFrame(),
                                                        () -> 0L,
                                                        verticalFOV,
                                                        imageWidth,
                                                        imageHeight,
                                                        minRange,
                                                        maxRange,
                                                        0.005,
                                                        0.005,
                                                        true,
                                                        publishRateHz);
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

            normalizedDepthImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC1);
            rgbaDepthImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC4);

            ffmpegLoggerDemoHelper.create(imageWidth, imageHeight, () ->
            {
               OpenCVTools.clampTo8BitUnsignedChar(l515.getLowLevelSimulator().getMetersDepthOpenCVMat(),
                                                   normalizedDepthImage.getBytedecoOpenCVMat(),
                                                   0.0,
                                                   255.0);
               OpenCVTools.convert8BitGrayTo8BitRGBA(normalizedDepthImage.getBytedecoOpenCVMat(), rgbaDepthImage.getBytedecoOpenCVMat());

               ffmpegLoggerDemoHelper.getLogger().put(rgbaDepthImage);
            });
         }

         @Override
         public void render()
         {
            l515.render(baseUI.getPrimaryScene());
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderImGuiWidgets()
         {
            ImGui.text("System native byte order: " + ByteOrder.nativeOrder().toString());
            ffmpegLoggerDemoHelper.renderImGuiBasicInfo();
            ImGui.text("Image dimensions: " + imageWidth + " x " + imageHeight);
            ffmpegLoggerDemoHelper.renderImGuiWidgets();
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
      new RDXFFMPEGL515DepthLoggingDemo();
   }
}