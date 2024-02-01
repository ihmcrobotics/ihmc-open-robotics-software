package us.ihmc.rdx.imgui;

import imgui.ImGui;
import imgui.type.ImFloat;

public class ImGuiSliderFloat extends ImGuiFancyWidget
{
   private final ImFloat imFloat;

   public ImGuiSliderFloat(String label, String format)
   {
      this(label, format, 0.0f);
   }

   public ImGuiSliderFloat(String label, String format, float initialValue)
   {
      super(label, format);
      imFloat = new ImFloat(initialValue);
   }

   /**
    * Shows the slider.
    */
   public boolean render(float minValue, float maxValue)
   {
      beforeWidgetRender();
      boolean valueChanged = ImGui.sliderFloat(label, imFloat.getData(), minValue, maxValue, format);
      afterWidgetRender();
      return valueChanged;
   }

   public void setFloatValue(float value)
   {
      imFloat.set(value);
   }

   public float getFloatValue()
   {
      return imFloat.get();
   }

   public ImFloat getImFloat()
   {
      return imFloat;
   }
}
