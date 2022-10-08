package us.ihmc.gdx.imgui;

import imgui.type.ImDouble;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class ImDoubleWrapper
{
   private final ImDouble imDouble = new ImDouble();
   private final DoubleSupplier wrappedValueGetter;
   private final DoubleConsumer wrappedValueSetter;

   public ImDoubleWrapper(DoubleSupplier wrappedValueGetter, DoubleConsumer wrappedValueSetter)
   {
      this.wrappedValueGetter = wrappedValueGetter;
      this.wrappedValueSetter = wrappedValueSetter;
   }

   public void accessImDouble(Consumer<ImDouble> imDoubleConsumer)
   {
      // This basic set has no effects to just set it even if the values are the same
      imDouble.set(wrappedValueGetter.getAsDouble());
      imDoubleConsumer.accept(imDouble);
      // wrappedValueSetter might be hooked to a callback, so let's prevent
      // that unless necessary
      double imDoubleValue = imDouble.get();
      if (imDoubleValue != wrappedValueGetter.getAsDouble())
      {
         wrappedValueSetter.accept(imDoubleValue);
      }
   }
}
