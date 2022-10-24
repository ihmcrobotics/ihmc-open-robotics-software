package us.ihmc.rdx.imgui;

import imgui.type.ImDouble;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertySetBasics;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class ImGuiDoubleWidget
{
   private final ImDoubleWrapper imDoubleWrapper;
   private final Consumer<ImDouble> renderImGuiWidget;

   public ImGuiDoubleWidget(StoredPropertySetBasics storedPropertySet, DoubleStoredPropertyKey key, Consumer<ImDouble> renderImGuiWidget)
   {
      this.renderImGuiWidget = renderImGuiWidget;
      imDoubleWrapper = new ImDoubleWrapper(storedPropertySet, key);
   }

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
