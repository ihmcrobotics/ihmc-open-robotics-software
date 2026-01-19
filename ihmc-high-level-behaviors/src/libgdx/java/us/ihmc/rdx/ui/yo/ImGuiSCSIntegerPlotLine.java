package us.ihmc.rdx.ui.yo;

import us.ihmc.rdx.simulation.scs2.RDXYoManager;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.function.Consumer;

public class ImGuiSCSIntegerPlotLine extends ImGuiSCSPlotLine
{
   private final YoInteger yoInteger;

   public ImGuiSCSIntegerPlotLine(YoInteger yoInteger, Consumer<YoVariable> removeSelf)
   {
      super(yoInteger, "0", removeSelf);
      this.yoInteger = yoInteger;
   }

   @Override
   public void setupLinkedVariable(RDXYoManager yoManager)
   {
   }
}
