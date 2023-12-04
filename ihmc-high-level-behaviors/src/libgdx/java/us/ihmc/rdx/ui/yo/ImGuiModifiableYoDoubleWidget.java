package us.ihmc.rdx.ui.yo;

import imgui.flag.ImGuiInputTextFlags;
import imgui.internal.ImGui;
import us.ihmc.behaviors.tools.yo.YoDoubleClientHelper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

public class ImGuiModifiableYoDoubleWidget
{
   private final YoDoubleClientHelper yoDoubleClientHelper;
   private final ImGuiModifiableYoDouble modifiableYoDouble;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   public ImGuiModifiableYoDoubleWidget(RDXYoVariableClientPanel yoVariableClientPanel, String yoVariableName)
   {
      yoDoubleClientHelper = yoVariableClientPanel.getYoVariableClientHelper().subscribeToYoDouble(yoVariableName);
      modifiableYoDouble = new ImGuiModifiableYoDouble(yoDoubleClientHelper);
   }

   public void renderImGuiWidgets()
   {
      modifiableYoDouble.update();
      ImGui.pushItemWidth(110.0f);
      double step = 0.1;
      double stepFast = 0.0;
      String format = "%.4f";
      if (ImGui.inputDouble(labels.get(modifiableYoDouble.getYoDoubleHelper().getName()),
                            modifiableYoDouble.getImDouble(),
                            step,
                            stepFast,
                            format,
                            ImGuiInputTextFlags.EnterReturnsTrue))
      {
         modifiableYoDouble.set();
      }
      ImGui.popItemWidth();
      ImGui.sameLine();
      ImGui.text("Server: " + modifiableYoDouble.getYoDoubleHelper().get());
   }
}
