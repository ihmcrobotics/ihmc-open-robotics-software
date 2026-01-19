package us.ihmc.rdx.ui.yo;

import us.ihmc.rdx.simulation.scs2.RDXYoManager;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.function.Consumer;

public class ImGuiSCSEnumPlotLine extends ImGuiSCSPlotLine
{
   private final YoEnum yoEnum;

   public ImGuiSCSEnumPlotLine(YoEnum yoEnum, Consumer<YoVariable> removeSelf)
   {
      super(yoEnum, "", removeSelf);
      this.yoEnum = yoEnum;
   }

   @Override
   public void setupLinkedVariable(RDXYoManager yoManager)
   {
   }
}
