package us.ihmc.rdx.imgui;

import imgui.type.ImFloat;

import java.util.function.Consumer;

/**
 * Syncs ImGui data with external data, provided as a StoredProperty or a Supplier and Consumer.
 * The user can rely on the data always being in sync due to the design of the access method.
 */

public class ImFloatWrapper
{
   private final ImFloat imFloat = new ImFloat();
   private final FloatSupplier wrappedValueGetter;
   private final FloatConsumer wrappedValueSetter;
   private final Consumer<ImFloat> widgetRenderer;
   private boolean changed = false;

   /**
    * @param wrappedValueGetter used for getting the underlying value
    * @param wrappedValueSetter used for setting the underlying value
    * @param widgetRenderer is used for rendering ImGui widgets with the ImGui
    * type provided to the given Consumer. This way, this class can ensure it
    * is synced to the external data before and after the widget is rendered
    * and modified by the ImGui user.
    */
   public ImFloatWrapper(FloatSupplier wrappedValueGetter, FloatConsumer wrappedValueSetter, Consumer<ImFloat> widgetRenderer)
   {
      this.wrappedValueGetter = wrappedValueGetter;
      this.wrappedValueSetter = wrappedValueSetter;
      this.widgetRenderer = widgetRenderer;
   }

   public void renderImGuiWidget()
   {
      // This basic set has no effects to just set it even if the values are the same
      imFloat.set(wrappedValueGetter.getAsFloat());
      widgetRenderer.accept(imFloat);
      // wrappedValueSetter might be hooked to a callback, so let's prevent
      // that unless necessary
      float imFloatValue = imFloat.get();
      changed = imFloatValue != wrappedValueGetter.getAsFloat();
      if (changed)
      {
         wrappedValueSetter.accept(imFloatValue);
      }
   }

   public boolean changed()
   {
      return changed;
   }

   @FunctionalInterface
   public interface FloatSupplier
   {
      float getAsFloat();
   }

   @FunctionalInterface
   public interface FloatConsumer
   {
      void accept(float value);
   }
}
