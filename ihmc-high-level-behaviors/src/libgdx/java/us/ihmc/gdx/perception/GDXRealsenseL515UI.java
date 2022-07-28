package us.ihmc.gdx.perception;

import boofcv.struct.calib.CameraPinholeBrown;
import imgui.ImGui;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.bytedeco.librealsense2.global.realsense2;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.GDXPointCloudRenderer;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.perception.*;
import us.ihmc.perception.realsense.BytedecoRealsense;
import us.ihmc.perception.realsense.RealSenseHardwareManager;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.time.FrequencyCalculator;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.nio.ByteOrder;

public class GDXRealsenseL515UI
{
   private static final String SERIAL_NUMBER = System.getProperty("l515.serial.number", "F1121365");
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/main/resources");
   private final Activator nativesLoadedActivator;
   private final GDXPose3DGizmo cameraPoseGizmo = new GDXPose3DGizmo();
   private YoRegistry yoRegistry = new YoRegistry(getClass().getSimpleName());
   private YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private RealSenseHardwareManager realSenseHardwareManager;
   private BytedecoRealsense l515;
   private GDXCVImagePanel depthImagePanel;
   private GDXCVImagePanel colorImagePanel;
   private Mat depthU16C1Image;
   private Mat color8UC3Image;
   private BytedecoImage depth32FC1Image;
   private FrequencyCalculator frameReadFrequency = new FrequencyCalculator();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImFloat laserPower = new ImFloat(100.0f);
   private final ImFloat receiverSensitivity = new ImFloat(0.5f);
   private final ImInt digitalGain = new ImInt(realsense2.RS2_DIGITAL_GAIN_LOW);
   private final String[] digitalGains = new String[] { "AUTO", "LOW", "HIGH" };
   private GDXPointCloudRenderer pointCloudRenderer;
   private final OpenCLManager openCLManager = new OpenCLManager();
   private final FramePoint3D framePoint = new FramePoint3D();

   public GDXRealsenseL515UI()
   {
      nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();

      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            ImGuiPanel panel = new ImGuiPanel("L515", this::renderImGuiWidgets);
            baseUI.getImGuiPanelManager().addPanel(panel);

            cameraPoseGizmo.create(baseUI.getPrimary3DPanel().getCamera3D());
            cameraPoseGizmo.setResizeAutomatically(true);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(cameraPoseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(cameraPoseGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(cameraPoseGizmo, GDXSceneLevel.VIRTUAL);
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  realSenseHardwareManager = new RealSenseHardwareManager(yoRegistry, yoGraphicsListRegistry);

                  l515 = realSenseHardwareManager.createFullFeaturedL515(SERIAL_NUMBER);
                  l515.enableColor(1280, 720, 30);
                  l515.initialize();

                  openCLManager.create();
                  pointCloudRenderer = new GDXPointCloudRenderer();
               }

               if (l515.readFrameData())
               {
                  l515.updateDataBytePointers();

                  if (depthImagePanel == null)
                  {
                     MutableBytePointer depthFrameData = l515.getDepthFrameData();
                     depthU16C1Image = new Mat(l515.getDepthHeight(), l515.getDepthWidth(), opencv_core.CV_16UC1, depthFrameData);

                     depth32FC1Image = new BytedecoImage(l515.getDepthWidth(), l515.getDepthHeight(), opencv_core.CV_32FC1);
                     depthImagePanel = new GDXCVImagePanel("L515 Depth", l515.getDepthWidth(), l515.getDepthHeight());
                     baseUI.getImGuiPanelManager().addPanel(depthImagePanel.getVideoPanel());

                     MutableBytePointer colorFrameData = l515.getColorFrameData();
                     color8UC3Image = new Mat(l515.getColorHeight(), l515.getColorWidth(), opencv_core.CV_8UC3, colorFrameData);
                     colorImagePanel = new GDXCVImagePanel("L515 Color", l515.getColorWidth(), l515.getColorHeight());
                     baseUI.getImGuiPanelManager().addPanel(colorImagePanel.getVideoPanel());

                     baseUI.getPerspectiveManager().reloadPerspective();

                     pointCloudRenderer.create(l515.getDepthWidth() * l515.getDepthHeight());
                     baseUI.getPrimaryScene().addRenderableProvider(pointCloudRenderer, GDXSceneLevel.MODEL);
                  }

                  frameReadFrequency.ping();
                  depthU16C1Image.convertTo(depth32FC1Image.getBytedecoOpenCVMat(), opencv_core.CV_32FC1, l515.getDepthToMeterConversion(), 0.0);

                  depthImagePanel.drawFloatImage(depth32FC1Image.getBytedecoOpenCVMat());

                  opencv_imgproc.cvtColor(color8UC3Image, colorImagePanel.getBytedecoImage().getBytedecoOpenCVMat(), opencv_imgproc.COLOR_RGB2RGBA);
                  colorImagePanel.draw();

                  pointCloudRenderer.prepareVertexBufferForAddingPoints();
                  CameraPinholeBrown depthCameraIntrinsics = l515.getDepthCameraIntrinsics();

                  // TODO: Put in OpenCL
                  for (int x = 0; x < l515.getDepthWidth(); x++)
                  {
                     for (int y = 0; y < l515.getDepthHeight(); y++)
                     {
                        float eyeDepth = depth32FC1Image.getFloat(x, y);

                        framePoint.setToZero(cameraPoseGizmo.getGizmoFrame());
                        framePoint.setX(eyeDepth);
                        framePoint.setY(-(x - depthCameraIntrinsics.getCx()) / depthCameraIntrinsics.getFx() * eyeDepth);
                        framePoint.setZ(-(y - depthCameraIntrinsics.getCy()) / depthCameraIntrinsics.getFy() * eyeDepth);

                        framePoint.changeFrame(ReferenceFrame.getWorldFrame());

                        pointCloudRenderer.putVertex(framePoint);
                     }
                  }

                  pointCloudRenderer.updateMeshFastest();
               }
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderImGuiWidgets()
         {
            ImGui.text("System native byte order: " + ByteOrder.nativeOrder().toString());

            if (depthImagePanel != null)
            {
               ImGui.text("Depth frame data size: " + l515.getDepthFrameDataSize());
               ImGui.text("Frame read frequency: " + frameReadFrequency.getFrequency());
               ImGui.text("Depth to meters conversion: " + l515.getDepthToMeterConversion());

               if (ImGui.sliderFloat(labels.get("Laser power"), laserPower.getData(), 0.0f, 100.0f))
               {
                  l515.setLaserPower(laserPower.get());
               }

               if (ImGui.combo(labels.get("Digital gain"), digitalGain, digitalGains))
               {
                  if (digitalGain.get() == 0)
                  {
                     l515.setDigitalGain(realsense2.RS2_DIGITAL_GAIN_AUTO);
                  }
                  else if (digitalGain.get() == 1)
                  {
                     l515.setDigitalGain(realsense2.RS2_DIGITAL_GAIN_LOW);
                  }
                  else
                  {
                     l515.setDigitalGain(realsense2.RS2_DIGITAL_GAIN_HIGH);
                  }

               }

               ImGui.text("Unsigned 16 Depth:");

               for (int i = 0; i < 5; i++)
               {
                  ImGui.text(depthU16C1Image.ptr(0, i).getShort() + " ");
               }

               ImGui.text("Float 32 Meters:");

               depth32FC1Image.rewind();
               for (int i = 0; i < 5; i++)
               {
                  ImGui.text(depth32FC1Image.getBackingDirectByteBuffer().getFloat() + " ");
               }

               ImGui.text("Depth R G B A:");

               depthImagePanel.getBytedecoImage().rewind();
               for (int i = 0; i < 5; i++)
               {
                  printBytes(depthImagePanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(0, i).get(0),
                             depthImagePanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(0, i).get(1),
                             depthImagePanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(0, i).get(2),
                             depthImagePanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(0, i).get(3));
               }

               ImGui.text("Color R G B:");

               for (int i = 0; i < 5; i++)
               {
                  printBytes(color8UC3Image.ptr(0, i).get(0),
                             color8UC3Image.ptr(0, i).get(1),
                             color8UC3Image.ptr(0, i).get(2));
               }

               ImGui.text("Color R G B A:");

               colorImagePanel.getBytedecoImage().rewind();
               for (int i = 0; i < 5; i++)
               {
                  printBytes(colorImagePanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(0, i).get(0),
                             colorImagePanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(0, i).get(1),
                             colorImagePanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(0, i).get(2),
                             colorImagePanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(0, i).get(3));
               }
            }
         }

         private void printBytes(byte byte0, byte byte1, byte byte2, byte byte3)
         {
            printInts(Byte.toUnsignedInt(byte0),
                      Byte.toUnsignedInt(byte1),
                      Byte.toUnsignedInt(byte2),
                      Byte.toUnsignedInt(byte3));
         }

         private void printBytes(byte byte0, byte byte1, byte byte2)
         {
            printInts(Byte.toUnsignedInt(byte0),
                      Byte.toUnsignedInt(byte1),
                      Byte.toUnsignedInt(byte2));
         }

         private void printInts(int int0, int int1, int int2, int int3)
         {
            ImGui.text(int0 + " " + int1 + " " + int2 + " " + int3);
         }

         private void printInts(int int0, int int1, int int2)
         {
            ImGui.text(int0 + " " + int1 + " " + int2);
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
            l515.deleteDevice();
            realSenseHardwareManager.deleteContext();
         }
      });
   }

   public static void main(String[] args)
   {
      new GDXRealsenseL515UI();
   }
}
