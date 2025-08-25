package us.ihmc.rdx.ui.graphics;

import imgui.ImGui;
import us.ihmc.rdx.imgui.ImGuiPlot;

/**
 * Used to detect dropped messages.
 */
public class RDXSequenceDiscontinuityPlot
{
   private long expectedNextSequenceNumber = -1;
   private long numberOfSequenceDiscontinuities = 0;
   private final ImGuiPlot sequenceDiscontinuitiesPlot = new ImGuiPlot("Sequence discontinuities", 1000, 230, 20);

   public RDXSequenceDiscontinuityPlot()
   {
      sequenceDiscontinuitiesPlot.hideLabel();
   }

   public void update(long sequenceNumber)
   {
      if (expectedNextSequenceNumber > 0 && sequenceNumber != expectedNextSequenceNumber)
      {
         numberOfSequenceDiscontinuities++;
      }
      expectedNextSequenceNumber = sequenceNumber + 1;
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Sequence discontinuities");
      ImGui.sameLine();
      sequenceDiscontinuitiesPlot.setWidth((int) ImGui.getColumnWidth());
      sequenceDiscontinuitiesPlot.render(numberOfSequenceDiscontinuities);
   }
}
