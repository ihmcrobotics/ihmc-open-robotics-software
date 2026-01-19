package us.ihmc.rdx.ui.yo;

import us.ihmc.rdx.simulation.scs2.RDXYoManager;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.function.Consumer;

public abstract class ImGuiSCSPlotLine
{
   private final YoVariable yoVariable;
   private final Consumer<YoVariable> removeSelf;

   public ImGuiSCSPlotLine(YoVariable yoVariable, String initialValueString, Consumer<YoVariable> removeSelf)
   {
      this.yoVariable = yoVariable;
      this.removeSelf = removeSelf;
   }

   public abstract void setupLinkedVariable(RDXYoManager yoManager);

   public void render(float plotWidth, float plotHeight, int lineIndex)
   {

   }

   public String getVariableName()
   {
      return yoVariable.getName();
   }

   public String getValueString(int bufferIndex)
   {
      return null;
   }
}
