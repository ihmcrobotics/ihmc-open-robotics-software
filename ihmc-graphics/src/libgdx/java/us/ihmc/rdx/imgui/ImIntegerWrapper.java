package us.ihmc.rdx.imgui;

import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertySetBasics;

import java.util.function.*;

/**
 * Syncs ImGui data with external data, provided as a StoredProperty or a Supplier and Consumer.
 * The user can rely on the data always being in sync due to the design of the access method.
 */
public class ImIntegerWrapper
{
   private static final String[] PASCAL_CASED_SIDE_NAMES = new String[] {"Left", "Right"};

   private final ImInt imInt = new ImInt();
   private final IntSupplier wrappedValueGetter;
   private final IntConsumer wrappedValueSetter;
   private final Consumer<ImInt> widgetRenderer;
   private boolean changed = false;

   /**
    * Convenience method for wrapping a StoredPropertySet key.
    */
   public ImIntegerWrapper(StoredPropertySetBasics storedPropertySet, IntegerStoredPropertyKey key, Consumer<ImInt> widgetRenderer)
   {
      this(() -> storedPropertySet.get(key), integerValue -> storedPropertySet.set(key, integerValue), widgetRenderer);
   }

   /**
    * Convenience method for wrapping a RobotSide.
    */
   public ImIntegerWrapper(Supplier<RobotSide> robotSideGetter, Consumer<RobotSide> robotSideSetter, String label)
   {
      this(() -> robotSideGetter.get().ordinal(),
           ordinal -> robotSideSetter.accept(RobotSide.values[ordinal]),
           imInt -> ImGui.combo(label, imInt, PASCAL_CASED_SIDE_NAMES));
   }

   /**
    * @param wrappedValueGetter used for getting the underlying value
    * @param wrappedValueSetter used for setting the underlying value
    * @param widgetRenderer is used for rendering ImGui widgets with the ImGui
    * type provided to the given Consumer. This way, this class can ensure it
    * is synced to the external data before and after the widget is rendered
    * and modified by the ImGui user.
    */
   public ImIntegerWrapper(IntSupplier wrappedValueGetter, IntConsumer wrappedValueSetter, Consumer<ImInt> widgetRenderer)
   {
      this.wrappedValueGetter = wrappedValueGetter;
      this.wrappedValueSetter = wrappedValueSetter;
      this.widgetRenderer = widgetRenderer;
   }

   public void renderImGuiWidget()
   {
      // This basic set has no effects to just set it even if the values are the same
      imInt.set(wrappedValueGetter.getAsInt());
      widgetRenderer.accept(imInt);
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
