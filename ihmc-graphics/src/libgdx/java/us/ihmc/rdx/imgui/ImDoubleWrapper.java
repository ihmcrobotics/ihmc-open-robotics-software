package us.ihmc.rdx.imgui;

import imgui.type.ImDouble;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertySetBasics;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * Syncs ImGui data with external data, provided as a StoredProperty or a Supplier and Consumer.
 * The user can rely on the data always being in sync due to the design of the access method.
 */

public class ImDoubleWrapper
{
   private final ImDouble imDouble = new ImDouble();
   private final DoubleSupplier wrappedValueGetter;
   private final DoubleConsumer wrappedValueSetter;
   private final Consumer<ImDouble> widgetRenderer;
   private boolean changed = false;

   /**
    * Convenience method for wrapping a StoredPropertySet key.
    */
   public ImDoubleWrapper(StoredPropertySetBasics storedPropertySet, DoubleStoredPropertyKey key, Consumer<ImDouble> widgetRenderer)
   {
      this(() -> storedPropertySet.get(key), doubleValue -> storedPropertySet.set(key, doubleValue), widgetRenderer);
   }

   /**
    * @param wrappedValueGetter used for getting the underlying value
    * @param wrappedValueSetter used for setting the underlying value
    * @param widgetRenderer is used for rendering ImGui widgets with the ImGui
    * type provided to the given Consumer. This way, this class can ensure it
    * is synced to the external data before and after the widget is rendered
    * and modified by the ImGui user.
    */
   public ImDoubleWrapper(DoubleSupplier wrappedValueGetter, DoubleConsumer wrappedValueSetter, Consumer<ImDouble> widgetRenderer)
   {
      this.wrappedValueGetter = wrappedValueGetter;
      this.wrappedValueSetter = wrappedValueSetter;
      this.widgetRenderer = widgetRenderer;
   }

   public void renderImGuiWidget()
   {
      // This basic set has no effects to just set it even if the values are the same
      imDouble.set(wrappedValueGetter.getAsDouble());
      widgetRenderer.accept(imDouble);
      // wrappedValueSetter might be hooked to a callback, so let's prevent
      // that unless necessary
      double imDoubleValue = imDouble.get();
      changed = imDoubleValue != wrappedValueGetter.getAsDouble();
      if (changed)
      {
         wrappedValueSetter.accept(imDoubleValue);
      }
   }

   public boolean changed()
   {
      return changed;
   }
}
