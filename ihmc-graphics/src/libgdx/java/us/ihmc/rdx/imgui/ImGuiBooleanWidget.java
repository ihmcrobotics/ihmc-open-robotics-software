package us.ihmc.rdx.imgui;

import imgui.type.ImBoolean;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class ImGuiBooleanWidget
{
   private final ImBooleanWrapper imBooleanWrapper;
   private Consumer<ImBoolean> renderImGuiWidget;

   public ImGuiBooleanWidget(Supplier<Boolean> wrappedValueGetter, Consumer<Boolean> wrappedValueSetter, Consumer<ImBoolean> renderImGuiWidget)
   {
      this.renderImGuiWidget = renderImGuiWidget;
      imBooleanWrapper = new ImBooleanWrapper(wrappedValueGetter, wrappedValueSetter);
   }

   public void renderImGuiWidget()
   {
      imBooleanWrapper.accessImBoolean(renderImGuiWidget);
   }
}
