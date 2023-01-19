package us.ihmc.rdx.imgui;

import imgui.type.ImInt;

import java.util.function.*;

public class ImGuiIntegerWidget
{
   private final ImIntegerWrapper imIntegerWrapper;
   private Consumer<ImInt> renderImGuiWidget;

   public ImGuiIntegerWidget(IntSupplier wrappedValueGetter, IntConsumer wrappedValueSetter, Consumer<ImInt> renderImGuiWidget)
   {
      this.renderImGuiWidget = renderImGuiWidget;
      imIntegerWrapper = new ImIntegerWrapper(wrappedValueGetter, wrappedValueSetter);
   }

   public void renderImGuiWidget()
   {
      imIntegerWrapper.accessImInt(renderImGuiWidget);
   }
}
