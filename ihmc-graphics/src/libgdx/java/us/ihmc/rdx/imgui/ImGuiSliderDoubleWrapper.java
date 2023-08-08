package us.ihmc.rdx.imgui;

import imgui.type.ImDouble;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class ImGuiSliderDoubleWrapper extends ImGuiFancyWidget
{
   private final ImDoubleWrapper imDoubleWrapper;

   public ImGuiSliderDoubleWrapper(String label,
                                   String format,
                                   double minValue,
                                   double maxValue,
                                   DoubleSupplier wrappedValueGetter,
                                   DoubleConsumer wrappedValueSetter)
   {
      super(label, format);
      imDoubleWrapper = new ImDoubleWrapper(wrappedValueGetter, wrappedValueSetter, imDouble -> render(minValue, maxValue, imDouble));
   }

   private void render(double minValue, double maxValue, ImDouble imDouble)
   {
      beforeWidgetRender();
      ImGuiTools.sliderDouble(label, imDouble, minValue, maxValue, format);
      afterWidgetRender();
   }

   public void render()
   {
      imDoubleWrapper.renderImGuiWidget();
   }
}
