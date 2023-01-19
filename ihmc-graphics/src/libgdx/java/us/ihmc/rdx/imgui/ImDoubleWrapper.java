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
   private boolean changed = false;

   public ImDoubleWrapper(StoredPropertySetBasics storedPropertySet, DoubleStoredPropertyKey key)
   {
      this(() -> storedPropertySet.get(key), doubleValue -> storedPropertySet.set(key, doubleValue));
   }

   public ImDoubleWrapper(DoubleSupplier wrappedValueGetter, DoubleConsumer wrappedValueSetter)
   {
      this.wrappedValueGetter = wrappedValueGetter;
      this.wrappedValueSetter = wrappedValueSetter;
   }

   /**
    * This access method is used for rendering ImGui widgets with the ImGui
    * type provided to the given Consumer. This way, this class can ensure it
    * is synced to the external data before and after the widget is rendered
    * and modified by the ImGui user.
    */
   public void accessImDouble(Consumer<ImDouble> imDoubleConsumer)
   {
      // This basic set has no effects to just set it even if the values are the same
      imDouble.set(wrappedValueGetter.getAsDouble());
      imDoubleConsumer.accept(imDouble);
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
