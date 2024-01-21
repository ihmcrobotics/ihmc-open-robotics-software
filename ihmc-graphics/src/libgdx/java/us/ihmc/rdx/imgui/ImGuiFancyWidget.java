package us.ihmc.rdx.imgui;

import imgui.ImGui;
import imgui.flag.ImGuiCol;

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
   /** Used by a lot of widgets that extend this class. Just here for brevity for those.*/
   protected final String format;
   private final String prefixLabel;
   /** Negative values fill available width. */
   private float widgetWidth = -1.0f;
   private boolean widgetTextColoring = false;
   private int widgetTextColor = 0;
   private String buttonText;
   private Runnable onButtonPressed;

   protected ImGuiFancyWidget(String label)
   {
      this.prefixLabel = label;
      this.label = labels.getHidden(label);
      this.format = null;
   }

   protected ImGuiFancyWidget(String label, String format)
   {
      this.prefixLabel = label;
      this.label = labels.getHidden(label);
      this.format = format;
   }

   protected void beforeWidgetRender()
   {
      ImGui.text(prefixLabel);
      ImGui.sameLine();

      float itemWidth = ImGui.getColumnWidth();
      if (widgetWidth >= 0.0f)
         itemWidth = widgetWidth;
      if (buttonText != null)
         itemWidth -= ImGuiTools.calcButtonWidth(buttonText) + ImGui.getStyle().getItemSpacingX();

      ImGui.pushItemWidth(itemWidth);

      if (widgetTextColoring)
         ImGui.pushStyleColor(ImGuiCol.Text, widgetTextColor);
   }

   protected void afterWidgetRender()
   {
      ImGui.popItemWidth();

      if (widgetTextColoring)
         ImGui.popStyleColor();

      if (buttonText != null)
      {
         ImGui.sameLine();
         if (ImGui.button(labels.get(buttonText, prefixLabel)))
            onButtonPressed.run();
      }
   }

   /**
    * Colors any text rendered in the widget, but not the label.
    * Call anytime. No need to call every tick. It will persist for this widget
    * until another color is set or the color is cleared using {@link #clearWidgetTextColor}.
    * Use ImGuiTools contants for colors. i.e. ImGuiTools.RED
    * @param intColor
    */
   public void setWidgetTextColor(int intColor)
   {
      widgetTextColoring = true;
      widgetTextColor = intColor;
   }

   /**
    * Go back to using the default style text color for the widget.
    * No need to call every tick. It will persist for this widget until another color is set.
    */
   public void clearWidgetTextColor()
   {
      widgetTextColoring = false;
   }

   public void setWidgetWidth(float widgetWidth)
   {
      this.widgetWidth = widgetWidth;
   }

   public void addButton(String buttonText, Runnable onButtonPressed)
   {
      this.buttonText = buttonText;
      this.onButtonPressed = onButtonPressed;
   }
}
