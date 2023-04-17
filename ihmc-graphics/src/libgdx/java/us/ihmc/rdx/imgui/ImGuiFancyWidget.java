package us.ihmc.rdx.imgui;

import imgui.ImGui;
import imgui.ImVec2;

/**
 * This class removes per-tick hash map lookup for the label, doing it once in constructor.
 * It also removes any per-tick String building that may be accidental normally, because
 * whatever String gets built just happens once and gets passed to the constructor.
 *
 * Since we make a class anyway, we go ahead and make the widget look a little nicer,
 * putting the label on the left and fitting the widget in the remaining available space,
 * which makes the names always readable if possible and the tuner + and - buttons aligned
 * vertically if there are multiple of these in a row in the panel.
 */
public abstract class ImGuiFancyWidget
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   protected final String label;
   protected final String format;
   private final String prefixLabel;
   private float prefixTextWidth;
   private boolean textWidthCalculated = false;

   protected ImGuiFancyWidget(String label, String format)
   {
      this.prefixLabel = label;
      this.label = labels.getHidden(label);
      this.format = format;
   }

   protected void beforeWidgetRender()
   {
      if (!textWidthCalculated)
      {
         textWidthCalculated = true;
         ImVec2 size = new ImVec2();
         ImGui.calcTextSize(size, prefixLabel);
         prefixTextWidth = size.x;
      }

      ImGui.text(prefixLabel);
      ImGui.sameLine();
      ImGui.pushItemWidth(ImGuiTools.getUsableWindowWidth() - prefixTextWidth);
   }

   protected void afterWidgetRender()
   {
      ImGui.popItemWidth();
   }
}
