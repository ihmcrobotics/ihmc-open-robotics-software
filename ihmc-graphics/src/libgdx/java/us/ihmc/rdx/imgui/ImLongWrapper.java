package us.ihmc.rdx.imgui;

import imgui.type.ImLong;

import java.util.function.Consumer;
import java.util.function.LongConsumer;
import java.util.function.LongSupplier;

/**
 * Syncs ImGui data with external data, provided as a StoredProperty or a Supplier and Consumer.
 * The user can rely on the data always being in sync due to the design of the access method.
 */
public class ImLongWrapper
{
   private final ImLong imLong = new ImLong();
   private final LongSupplier wrappedValueGetter;
   private final LongConsumer wrappedValueSetter;
   private final Consumer<ImLong> widgetRenderer;
   private boolean changed = false;

   /**
    * @param wrappedValueGetter used for getting the underlying value
    * @param wrappedValueSetter used for setting the underlying value
    * @param widgetRenderer is used for rendering ImGui widgets with the ImGui
    * type provided to the given Consumer. This way, this class can ensure it
    * is synced to the external data before and after the widget is rendered
    * and modified by the ImGui user.
    */
   public ImLongWrapper(LongSupplier wrappedValueGetter, LongConsumer wrappedValueSetter, Consumer<ImLong> widgetRenderer)
   {
      this.wrappedValueGetter = wrappedValueGetter;
      this.wrappedValueSetter = wrappedValueSetter;
      this.widgetRenderer = widgetRenderer;
   }

   public void renderImGuiWidget()
   {
      // This basic set has no effects to just set it even if the values are the same
      imLong.set(wrappedValueGetter.getAsLong());
      widgetRenderer.accept(imLong);
      // wrappedValueSetter might be hooked to a callback, so let's prevent
      // that unless necessary
      long imLongValue = imLong.get();
      changed = imLongValue != wrappedValueGetter.getAsLong();
      if (changed)
      {
         wrappedValueSetter.accept(imLongValue);
      }
   }

   public boolean changed()
   {
      return changed;
   }
}
