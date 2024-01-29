package us.ihmc.rdx.imgui;

import imgui.type.ImDouble;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class ImGuiSliderDoubleWrapper extends ImGuiFancyWidget
{
   private final ImDoubleWrapper imDoubleWrapper;
   private final Runnable onUserModified;

   public ImGuiSliderDoubleWrapper(String label,
                                   String format,
                                   double minValue,
                                   double maxValue,
                                   DoubleSupplier wrappedValueGetter,
                                   DoubleConsumer wrappedValueSetter)
   {
      this(label, format, minValue, maxValue, wrappedValueGetter, wrappedValueSetter, () -> { });
   }

   public ImGuiSliderDoubleWrapper(String label,
                                   String format,
                                   double minValue,
                                   double maxValue,
                                   DoubleSupplier wrappedValueGetter,
                                   DoubleConsumer wrappedValueSetter,
                                   Runnable onUserModified)
   {
      super(label, format);
      this.onUserModified = onUserModified;
      imDoubleWrapper = new ImDoubleWrapper(wrappedValueGetter, wrappedValueSetter, imDouble -> render(minValue, maxValue, imDouble));
   }

   private void render(double minValue, double maxValue, ImDouble imDouble)
   {
      if (ImGuiTools.sliderDouble(label, imDouble, minValue, maxValue, format))
         onUserModified.run();
   }

   public void renderImGuiWidget()
   {
      beforeWidgetRender();
      imDoubleWrapper.renderImGuiWidget();
      afterWidgetRender();
   }
}
