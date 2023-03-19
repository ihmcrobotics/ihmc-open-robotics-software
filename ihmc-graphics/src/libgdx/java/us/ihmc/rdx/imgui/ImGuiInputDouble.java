package us.ihmc.rdx.imgui;

import imgui.ImGui;
import imgui.ImVec2;
import imgui.type.ImDouble;

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
public class ImGuiInputDouble
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final String label;
   private final String format;
   private final String prefixLabel;
   private float prefixTextWidth;
   private boolean textWidthCalculated = false;
   private final ImDouble imDouble;

   public ImGuiInputDouble(String label, String format)
   {
      this(label, format, 0.0);
   }

   public ImGuiInputDouble(String label, String format, double initialValue)
   {
      this.prefixLabel = label;
      this.label = labels.getHidden(label);
      this.format = format;
      imDouble = new ImDouble(initialValue);
   }

   /**
    * Won't show the + and - buttons.
    */
   public void render()
   {
      render(0.0, 0.0);
   }

   /**
    * Shows the + and - buttons.
    * @param step normal step
    * @param stepFast step when holding ctrl key and clicking + and - buttons
    */
   public boolean render(double step, double stepFast)
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

      boolean valueChanged = ImGuiTools.volatileInputDouble(label, imDouble, step, stepFast, format);

      ImGui.popItemWidth();

      return valueChanged;
   }

   public void setDoubleValue(double value)
   {
      imDouble.set(value);
   }

   public double getDoubleValue()
   {
      return imDouble.get();
   }

   public ImDouble getImDouble()
   {
      return imDouble;
   }
}
