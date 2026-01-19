package us.ihmc.rdx.ui.yo;

import us.ihmc.rdx.simulation.scs2.RDXYoManager;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.function.Consumer;

public class ImGuiSCSDoublePlotLine extends ImGuiSCSPlotLine
{
   private final YoDouble yoDouble;

   public ImGuiSCSDoublePlotLine(YoDouble yoDouble, Consumer<YoVariable> removeSelf)
   {
      super(yoDouble, "NaN", removeSelf);
      this.yoDouble = yoDouble;
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
