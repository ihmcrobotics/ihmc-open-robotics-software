package us.ihmc.rdx.imgui;

import imgui.type.ImBoolean;
import us.ihmc.tools.property.BooleanStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertySetBasics;

import java.util.function.*;

/**
 * Syncs ImGui data with external data, provided as a StoredProperty or a Supplier and Consumer.
 * The user can rely on the data always being in sync due to the design of the access method.
 */
public class ImBooleanWrapper
{
   private final ImBoolean imBoolean = new ImBoolean();
   private final Supplier<Boolean> wrappedValueGetter;
   private final Consumer<Boolean> wrappedValueSetter;
   private boolean changed = false;

   public ImBooleanWrapper(StoredPropertySetBasics storedPropertySet, BooleanStoredPropertyKey booleanKey)
   {
      this(() -> storedPropertySet.get(booleanKey), booleanValue -> storedPropertySet.set(booleanKey, booleanValue));
   }

   public ImBooleanWrapper(Supplier<Boolean> wrappedValueGetter, Consumer<Boolean> wrappedValueSetter)
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
   public void accessImBoolean(Consumer<ImBoolean> imBooleanConsumer)
   {
      // This basic set has no effects to just set it even if the values are the same
      imBoolean.set(wrappedValueGetter.get());
      imBooleanConsumer.accept(imBoolean);
      // wrappedValueSetter might be hooked to a callback, so let's prevent
      // that unless necessary
      boolean imBooleanValue = imBoolean.get();
      changed = imBooleanValue != wrappedValueGetter.get();
      if (changed)
      {
         wrappedValueSetter.accept(imBooleanValue);
      }
   }

   public boolean changed()
   {
      return changed;
   }
}
