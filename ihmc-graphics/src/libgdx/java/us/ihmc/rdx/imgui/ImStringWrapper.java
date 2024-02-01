package us.ihmc.rdx.imgui;

import imgui.type.ImString;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Syncs ImGui data with external data, provided as a Supplier and Consumer.
 * The user can rely on the data always being in sync due to the design of the access method.
 */
public class ImStringWrapper
{
   private final ImString imString = new ImString();
   private final Supplier<String> wrappedValueGetter;
   private final Consumer<String> wrappedValueSetter;
   private final Consumer<ImString> widgetRenderer;
   private boolean changed = false;

   /**
    * @param wrappedValueGetter used for getting the underlying value
    * @param wrappedValueSetter used for setting the underlying value
    * @param widgetRenderer is used for rendering ImGui widgets with the ImGui
    * type provided to the given Consumer. This way, this class can ensure it
    * is synced to the external data before and after the widget is rendered
    * and modified by the ImGui user.
    */
   public ImStringWrapper(Supplier<String> wrappedValueGetter, Consumer<String> wrappedValueSetter, Consumer<ImString> widgetRenderer)
   {
      this.wrappedValueGetter = wrappedValueGetter;
      this.wrappedValueSetter = wrappedValueSetter;
      this.widgetRenderer = widgetRenderer;
   }

   public void renderImGuiWidget()
   {
      // This basic set has no effects to just set it even if the values are the same
      imString.set(wrappedValueGetter.get());
      widgetRenderer.accept(imString);
      // wrappedValueSetter might be hooked to a callback, so let's prevent
      // that unless necessary
      String imStringValue = imString.get();
      changed = !imStringValue.equals(wrappedValueGetter.get());
      if (changed)
      {
         wrappedValueSetter.accept(imStringValue);
      }
   }

   public boolean changed()
   {
      return changed;
   }

   public String get()
   {
      return wrappedValueGetter.get();
   }

   public void set(String value)
   {
      wrappedValueSetter.accept(value);
   }
}
