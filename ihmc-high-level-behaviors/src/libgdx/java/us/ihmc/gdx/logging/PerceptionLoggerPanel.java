package us.ihmc.gdx.logging;

import us.ihmc.gdx.imgui.ImGuiPanel;

public class PerceptionLoggerPanel extends ImGuiPanel
{
   private PerceptionDataLogger logger;
   private PerceptionDataLoader loader;

   public PerceptionLoggerPanel(String panelName)
   {
      super(panelName);
   }
}
