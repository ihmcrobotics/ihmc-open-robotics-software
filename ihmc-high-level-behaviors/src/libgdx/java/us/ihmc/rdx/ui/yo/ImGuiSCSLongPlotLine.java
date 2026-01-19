package us.ihmc.rdx.ui.yo;

import us.ihmc.rdx.simulation.scs2.RDXYoManager;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.function.Consumer;

public class ImGuiSCSLongPlotLine extends ImGuiSCSPlotLine
{
   private final YoLong yoLong;

   public ImGuiSCSLongPlotLine(YoLong yoLong, Consumer<YoVariable> removeSelf)
   {
      super(yoLong, "0", removeSelf);
      this.yoLong = yoLong;
   }

   @Override
   public void setupLinkedVariable(RDXYoManager yoManager)
   {
   }

   @Override
   protected void plot(String labelID)
   {
   }

   @Override
   protected void update()
   {
   }
}
