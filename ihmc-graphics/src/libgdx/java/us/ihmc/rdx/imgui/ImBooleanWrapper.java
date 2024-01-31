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
   private final Consumer<ImBoolean> widgetRenderer;
   private boolean changed = false;

   /**
    * Convenience method for wrapping a StoredPropertySet key.
    */
   public ImBooleanWrapper(StoredPropertySetBasics storedPropertySet, BooleanStoredPropertyKey booleanKey, Consumer<ImBoolean> widgetRenderer)
   {
      this(() -> storedPropertySet.get(booleanKey), booleanValue -> storedPropertySet.set(booleanKey, booleanValue), widgetRenderer);
   }

   /**
    * @param wrappedValueGetter used for getting the underlying value
    * @param wrappedValueSetter used for setting the underlying value
    * @param widgetRenderer is used for rendering ImGui widgets with the ImGui
    * type provided to the given Consumer. This way, this class can ensure it
    * is synced to the external data before and after the widget is rendered
    * and modified by the ImGui user.
    */
   public ImBooleanWrapper(Supplier<Boolean> wrappedValueGetter, Consumer<Boolean> wrappedValueSetter, Consumer<ImBoolean> widgetRenderer)
   {
      this.wrappedValueGetter = wrappedValueGetter;
      this.wrappedValueSetter = wrappedValueSetter;
      this.widgetRenderer = widgetRenderer;
   }

   public void renderImGuiWidget()
   {
      // This basic set has no effects to just set it even if the values are the same
      imBoolean.set(wrappedValueGetter.get());
      widgetRenderer.accept(imBoolean);
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

   public boolean get()
   {
      return wrappedValueGetter.get();
   }

   public void set(boolean value)
   {
      wrappedValueSetter.accept(value);
   }
}
