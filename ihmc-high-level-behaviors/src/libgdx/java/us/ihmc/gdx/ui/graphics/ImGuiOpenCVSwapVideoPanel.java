package us.ihmc.gdx.ui.graphics;

import us.ihmc.gdx.imgui.ImGuiVideoPanel;
import us.ihmc.tools.thread.ZeroCopySwapReference;

public class ImGuiOpenCVSwapVideoPanel
{
   private final ImGuiVideoPanel videoPanel;
   private final ZeroCopySwapReference<ImGuiOpenCVSwapVideoPanelData> dataSwapReferenceManager;

   public ImGuiOpenCVSwapVideoPanel(String panelName, boolean flipY)
   {
      this.videoPanel = new ImGuiVideoPanel(panelName, flipY);
      dataSwapReferenceManager = new ZeroCopySwapReference<>(ImGuiOpenCVSwapVideoPanelData::new);
   }

   public ZeroCopySwapReference<ImGuiOpenCVSwapVideoPanelData> getDataSwapReferenceManager()
   {
      return dataSwapReferenceManager;
   }

   public ImGuiVideoPanel getVideoPanel()
   {
      return videoPanel;
   }
}
