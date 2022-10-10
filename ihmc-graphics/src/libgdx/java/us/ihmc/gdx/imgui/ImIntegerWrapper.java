package us.ihmc.gdx.imgui;

import imgui.type.ImInt;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertySetBasics;

import java.util.function.*;

public class ImIntegerWrapper
{
   private final ImInt imInt = new ImInt();
   private final IntSupplier wrappedValueGetter;
   private final IntConsumer wrappedValueSetter;

   public ImIntegerWrapper(StoredPropertySetBasics storedPropertySet, IntegerStoredPropertyKey key)
   {
      this(() -> storedPropertySet.get(key), integerValue -> storedPropertySet.set(key, integerValue));
   }

   public ImIntegerWrapper(IntSupplier wrappedValueGetter, IntConsumer wrappedValueSetter)
   {
      this.wrappedValueGetter = wrappedValueGetter;
      this.wrappedValueSetter = wrappedValueSetter;
   }

   public void accessImInt(Consumer<ImInt> imIntConsumer)
   {
      // This basic set has no effects to just set it even if the values are the same
      imInt.set(wrappedValueGetter.getAsInt());
      imIntConsumer.accept(imInt);
      // wrappedValueSetter might be hooked to a callback, so let's prevent
      // that unless necessary
      int imIntValue = imInt.get();
      if (imIntValue != wrappedValueGetter.getAsInt())
      {
         wrappedValueSetter.accept(imIntValue);
      }
   }
}
