package us.ihmc.rdx.ui.tools;

import imgui.ImGui;
import us.ihmc.rdx.imgui.RDXPanel;

/**
 * Legacy Fast DDS pub/sub statistics UI — not available after jros2 migration.
 * Stubbed so libgdx compiles; re-implement when jros2 exposes equivalent metrics.
 */
public class RDXROS2StatsPanel extends RDXPanel
{
   public RDXROS2StatsPanel()
   {
      super("ROS 2 Stats", null, false, true);
      setRenderMethod(this::renderImGuiWidgets);
   }

   private void renderImGuiWidgets()
   {
      ImGui.text("ROS 2 topic statistics are unavailable during the jros2 migration.");
   }
}
