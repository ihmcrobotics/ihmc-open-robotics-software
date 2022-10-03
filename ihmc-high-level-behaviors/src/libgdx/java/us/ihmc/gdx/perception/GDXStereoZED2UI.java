package us.ihmc.gdx.perception;

import imgui.ImGui;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_videoio.VideoCapture;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.interactable.GDXInteractableRealsenseD435;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.MutableBytePointer;
import us.ihmc.perception.realsense.BytedecoRealsense;
import us.ihmc.perception.realsense.RealSenseHardwareManager;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.time.FrequencyCalculator;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.nio.ByteOrder;

public class GDXStereoZED2UI
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/main/resources");
   private final Activator nativesLoadedActivator;
   private GDXCVImagePanel colorImagePanelLeft;
   private GDXCVImagePanel colorImagePanelRight;
   private Mat colorU8C3ImageLeft = new Mat(480, 640, opencv_core.CV_8UC3);
   private Mat colorU8C3ImageRight = new Mat(480, 640, opencv_core.CV_8UC3);;

   private VideoCapture cap;

   private FrequencyCalculator leftFrameReadFrequency = new FrequencyCalculator();
   private FrequencyCalculator rightFrameReadFrequency = new FrequencyCalculator();

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   public GDXStereoZED2UI()
   {
      nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();

      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            ImGuiPanel panel = new ImGuiPanel("D435", this::renderImGuiWidgets);
            baseUI.getImGuiPanelManager().addPanel(panel);

         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  cap = new VideoCapture(0);
               }

               if (cap.read(colorU16C3ImageLeft))
               {

                  if (depthImagePanel == null)
                  {
                     MutableBytePointer depthFrameData = d435.getDepthFrameData();

                     depthImagePanel = new GDXCVImagePanel("D435 Depth", d435.getDepthWidth(), d435.getDepthHeight());
                     baseUI.getImGuiPanelManager().addPanel(depthImagePanel.getVideoPanel());

                     baseUI.getPerspectiveManager().reloadPerspective();
                  }

                  frameReadFrequency.ping();

                  depthImagePanel.drawFloatImage(colorU16C3ImageLeft);
               }
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderImGuiWidgets()
         {
            ImGui.text("System native byte order: " + ByteOrder.nativeOrder().toString());

            if (colorImagePanelLeft != null)
            {
               ImGui.text("Left Frame read frequency: " + leftFrameReadFrequency.getFrequency());
            }

            if (colorImagePanelRight != null)
            {
               ImGui.text("Right Frame read frequency: " + rightFrameReadFrequency.getFrequency());
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
         }
      });
   }

   public static void main(String[] args)
   {
      new GDXStereoZED2UI();
   }
}
