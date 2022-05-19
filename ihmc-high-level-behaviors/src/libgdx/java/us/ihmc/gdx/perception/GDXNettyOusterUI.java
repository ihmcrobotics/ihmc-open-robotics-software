package us.ihmc.gdx.perception;

import imgui.ImGui;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.netty.NettyOuster;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.time.FrequencyCalculator;

import java.nio.ByteOrder;

public class GDXNettyOusterUI
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/main/resources");
   private final Activator nativesLoadedActivator;
   private NettyOuster ouster;
   private GDXCVImagePanel imagePanel;
   private final FrequencyCalculator frameReadFrequency = new FrequencyCalculator();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   public GDXNettyOusterUI()
   {
      nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();

      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            ImGuiPanel panel = new ImGuiPanel("Ouster", this::renderImGuiWidgets);
            baseUI.getImGuiPanelManager().addPanel(panel);
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  ouster = new NettyOuster();
                  ouster.start();
               }

               if (imagePanel == null && ouster.isInitialized())
               {
                  imagePanel = new GDXCVImagePanel("Ouster Depth Image", ouster.getImageWidth(), ouster.getImageHeight());

                  baseUI.getImGuiPanelManager().addPanel(imagePanel.getVideoPanel());
                  baseUI.getPerspectiveManager().reloadPerspective();
               }

               frameReadFrequency.ping();

               if (imagePanel != null)
               {
                  imagePanel.drawFloatImage(ouster.getBytedecoImage().getBytedecoOpenCVMat());
               }
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderImGuiWidgets()
         {
            ImGui.text("System native byte order: " + ByteOrder.nativeOrder().toString());

            if (imagePanel != null)
            {
               ImGui.text("Frame read frequency: " + frameReadFrequency.getFrequency());
            }
         }

         @Override
         public void dispose()
         {
            ouster.stop();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new GDXNettyOusterUI();
   }
}