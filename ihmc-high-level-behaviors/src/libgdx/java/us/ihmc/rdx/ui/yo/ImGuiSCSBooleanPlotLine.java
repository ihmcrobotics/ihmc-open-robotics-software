package us.ihmc.rdx.ui.yo;

import us.ihmc.rdx.simulation.scs2.RDXYoManager;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.function.Consumer;

public class ImGuiSCSBooleanPlotLine extends ImGuiSCSPlotLine
{
   private final YoBoolean yoBoolean;

   public ImGuiSCSBooleanPlotLine(YoBoolean yoBoolean, Consumer<YoVariable> removeSelf)
   {
      super(yoBoolean, "false", removeSelf);
      this.yoBoolean = yoBoolean;
   }

   @Override
   public void setupLinkedVariable(RDXYoManager yoManager)
   {
   }
}
