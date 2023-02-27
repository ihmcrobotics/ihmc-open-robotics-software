package us.ihmc.rdx.imgui;

import imgui.type.ImInt;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertySetBasics;

import java.util.function.*;

/**
 * Syncs ImGui data with external data, provided as a StoredProperty or a Supplier and Consumer.
 * The user can rely on the data always being in sync due to the design of the access method.
 */
public class ImIntegerWrapper
{
   private final ImInt imInt = new ImInt();
   private final IntSupplier wrappedValueGetter;
   private final IntConsumer wrappedValueSetter;
   private boolean changed = false;

   public ImIntegerWrapper(StoredPropertySetBasics storedPropertySet, IntegerStoredPropertyKey key)
   {
      this(() -> storedPropertySet.get(key), integerValue -> storedPropertySet.set(key, integerValue));
   }

   public ImIntegerWrapper(IntSupplier wrappedValueGetter, IntConsumer wrappedValueSetter)
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
   public void accessImInt(Consumer<ImInt> imIntConsumer)
   {
      // This basic set has no effects to just set it even if the values are the same
      imInt.set(wrappedValueGetter.getAsInt());
      imIntConsumer.accept(imInt);
      // wrappedValueSetter might be hooked to a callback, so let's prevent
      // that unless necessary
      int imIntValue = imInt.get();
      changed = imIntValue != wrappedValueGetter.getAsInt();
      if (changed)
      {
         wrappedValueSetter.accept(imIntValue);
      }
   }

   public boolean changed()
   {
      return changed;
   }
}
