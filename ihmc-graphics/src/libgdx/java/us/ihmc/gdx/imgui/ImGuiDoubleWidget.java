package us.ihmc.gdx.imgui;

import imgui.type.ImDouble;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class ImGuiDoubleWidget
{
   private final ImDoubleWrapper imDoubleWrapper;
   private Consumer<ImDouble> renderImGuiWidget;

   public ImGuiDoubleWidget(DoubleSupplier wrappedValueGetter, DoubleConsumer wrappedValueSetter, Consumer<ImDouble> renderImGuiWidget)
   {
      this.renderImGuiWidget = renderImGuiWidget;
      imDoubleWrapper = new ImDoubleWrapper(wrappedValueGetter, wrappedValueSetter);
   }

   public void renderImGuiWidget()
   {
      imDoubleWrapper.accessImDouble(renderImGuiWidget);
   }
}
