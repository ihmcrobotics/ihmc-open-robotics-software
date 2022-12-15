package us.ihmc.rdx.ui.graphics.live;

import us.ihmc.rdx.imgui.ImGuiPlot;

public class RDXSequenceDiscontinuityPlot
{
   private long expectedNextSequenceNumber = -1;
   private long numberOfSequenceDiscontinuities = 0;
   private final ImGuiPlot sequenceDiscontinuitiesPlot = new ImGuiPlot("Sequence discontinuities", 1000, 230, 20);

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
      sequenceDiscontinuitiesPlot.render(numberOfSequenceDiscontinuities);
   }
}
