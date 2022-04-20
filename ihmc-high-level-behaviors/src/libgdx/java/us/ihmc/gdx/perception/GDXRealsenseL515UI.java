package us.ihmc.gdx.perception;

import imgui.ImGui;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.realsense.NonRealtimeL515;
import us.ihmc.perception.realsense.RealSenseHardwareManager;
import us.ihmc.tools.thread.Activator;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.ShortBuffer;

public class GDXRealsenseL515UI
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/main/resources");
   private final Activator nativesLoadedActivator;
   private final GDXPose3DGizmo sensorPoseGizmo = new GDXPose3DGizmo();
   private YoRegistry yoRegistry = new YoRegistry(getClass().getSimpleName());
   private YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private RealSenseHardwareManager realSenseHardwareManager;
   private NonRealtimeL515 l515;
   private GDXCVImagePanel depthImagePanel;
   private BytedecoImage depthU16C1Image;
   private BytedecoImage depth32FC1Image;

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
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  realSenseHardwareManager = new RealSenseHardwareManager(yoRegistry, yoGraphicsListRegistry);

                  l515 = realSenseHardwareManager.createNonRealtimeL515("demo_", "F1121365");
                  l515.initialize();
               }

               l515.update();

               if (depthImagePanel == null)
               {
                  if (l515.getDepthByteBuffer() != null)
                  {
                     depthU16C1Image = new BytedecoImage(l515.getDepthWidth(), l515.getDepthHeight(), opencv_core.CV_16UC1, l515.getDepthByteBuffer());
                     depth32FC1Image = new BytedecoImage(l515.getDepthWidth(), l515.getDepthHeight(), opencv_core.CV_32FC1);
                     depthImagePanel = new GDXCVImagePanel("L515 Depth", l515.getDepthWidth(), l515.getDepthHeight());
                     baseUI.getImGuiPanelManager().addPanel(depthImagePanel.getVideoPanel());

                     baseUI.getPerspectiveManager().reloadPerspective();
                  }
               }

               if (depthU16C1Image != null)
               {
                  depthU16C1Image.rewind();
                  depth32FC1Image.rewind();
                  depthU16C1Image.getBytedecoOpenCVMat().convertTo(depth32FC1Image.getBytedecoOpenCVMat(),
                                                                   opencv_core.CV_32FC1,
                                                                   l515.getDepthToMeterConversion(),
                                                                   0.0);

                  depth32FC1Image.rewind();
                  depthU16C1Image.rewind();
                  depthImagePanel.drawFloatImage(depthU16C1Image.getBytedecoOpenCVMat());
               }
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderImGuiWidgets()
         {
            ImGui.text("System native byte order: " + ByteOrder.nativeOrder().toString());
            ImGui.text("Sensor depth buffer shorts:");

            if (l515 != null && depthU16C1Image != null)
            {
               ImGui.text("Depth to meters conversion: " + l515.getDepthToMeterConversion());

               ShortBuffer depthShortBuffer = l515.getDepthShortBuffer();
               if (depthShortBuffer != null)
               {
                  depthShortBuffer.rewind();
                  for (int i = 0; i < 10; i++)
                  {
                     ImGui.text(depthShortBuffer.get() + " ");
                  }

                  ImGui.text("Unsigned 16 Depth:");

                  depthU16C1Image.rewind();
                  for (int i = 0; i < 10; i++)
                  {
                     ImGui.text(depthU16C1Image.getBackingDirectByteBuffer().getShort() + " ");
                  }

                  ImGui.text("Float 32 Meters:");

                  depth32FC1Image.rewind();
                  for (int i = 0; i < 10; i++)
                  {
                     ImGui.text(depth32FC1Image.getBackingDirectByteBuffer().getFloat() + " ");
                  }

                  ImGui.text("R G B A:");

                  depthImagePanel.getBytedecoImage().rewind();
                  for (int i = 0; i < 4; i++)
                  {
                     printBytes(depthImagePanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(0, i).get(0),
                                depthImagePanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(0, i).get(1),
                                depthImagePanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(0, i).get(2),
                                depthImagePanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(0, i).get(3));
                  }
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

         private void printInts(int int0, int int1, int int2, int int3)
         {
            ImGui.text(int0 + " " + int1 + " " + int2 + " " + int3);
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
