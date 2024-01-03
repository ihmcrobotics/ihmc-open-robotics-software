package us.ihmc.rdx.simulation.scs2;

import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.scs2.sharedMemory.LinkedYoVariable;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoVariable;

public class RDXLinkedYoBooleanWidget
{
   private final LinkedYoVariable<YoBoolean> linkedYoVariable;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean imBoolean = new ImBoolean();

   public RDXLinkedYoBooleanWidget(RDXYoManager yoManager, String variableName)
   {
      YoVariable yoVariable = yoManager.getRootRegistry().findVariable(variableName);

      linkedYoVariable = (LinkedYoVariable<YoBoolean>) yoManager.newLinkedYoVariable(yoVariable);
      linkedYoVariable.addUser(this);
   }

   public void renderImGuiWidgets()
   {
      linkedYoVariable.pull();

      imBoolean.set(linkedYoVariable.getLinkedYoVariable().getValue());

      if (ImGui.checkbox(labels.get(linkedYoVariable.getLinkedYoVariable().getName()), imBoolean))
      {
         linkedYoVariable.getLinkedYoVariable().set(imBoolean.get());
         linkedYoVariable.push();
      }
   }
}
