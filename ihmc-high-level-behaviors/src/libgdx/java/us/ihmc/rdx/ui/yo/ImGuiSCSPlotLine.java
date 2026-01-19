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

   protected abstract void plot(String labelID);

   protected abstract void update();

   public boolean render()
   {
      return false;
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
