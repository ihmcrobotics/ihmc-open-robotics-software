package us.ihmc.rdx.ui.yo;

import imgui.internal.ImGui;
import us.ihmc.behaviors.tools.yo.YoBooleanClientHelper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

public class ImGuiModifiableYoBooleanWidget
{
   private final YoBooleanClientHelper yoBooleanClientHelper;
   private final ImGuiModifiableYoBoolean modifiableYoBoolean;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   public ImGuiModifiableYoBooleanWidget(ImGuiYoVariableClientUI yoVariableClientUI, String yoVariableName)
   {
      yoBooleanClientHelper = yoVariableClientUI.getYoVariableClientHelper().subscribeToYoBoolean(yoVariableName);
      modifiableYoBoolean = new ImGuiModifiableYoBoolean(yoBooleanClientHelper);
   }

   public void renderImGuiWidgets()
   {
      modifiableYoBoolean.update();
      if (ImGui.checkbox(labels.get(modifiableYoBoolean.getYoBooleanHelper().getName()), modifiableYoBoolean.getImBoolean()))
      {
         modifiableYoBoolean.set();
      }
      ImGui.sameLine();
      ImGui.text("Server: " + modifiableYoBoolean.getYoBooleanHelper().get());
   }
}
