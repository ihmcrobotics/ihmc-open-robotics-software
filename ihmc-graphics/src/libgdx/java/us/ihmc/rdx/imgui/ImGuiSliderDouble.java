package us.ihmc.rdx.imgui;

import imgui.type.ImDouble;

public class ImGuiSliderDouble extends ImGuiFancyWidget
{
   private final ImDouble imDouble;

   public ImGuiSliderDouble(String label, String format)
   {
      this(label, format, 0.0);
   }

   public ImGuiSliderDouble(String label, String format, double initialValue)
   {
      super(label, format);
      imDouble = new ImDouble(initialValue);
   }

   public ImGuiSliderDouble(String label, String format, ImDouble imDouble)
   {
      super(label, format);
      this.imDouble = imDouble;
   }

   /**
    * Shows the slider.
    */
   public boolean render(double minValue, double maxValue)
   {
      beforeWidgetRender();
      boolean valueChanged = ImGuiTools.sliderDouble(label, imDouble, minValue, maxValue, format);
      afterWidgetRender();
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
