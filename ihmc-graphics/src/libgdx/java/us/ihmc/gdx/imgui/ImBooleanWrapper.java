package us.ihmc.gdx.imgui;

import imgui.type.ImBoolean;

import java.util.function.*;

public class ImBooleanWrapper
{
   private final ImBoolean imBoolean = new ImBoolean();
   private final Supplier<Boolean> wrappedValueGetter;
   private final Consumer<Boolean> wrappedValueSetter;

   public ImBooleanWrapper(Supplier<Boolean> wrappedValueGetter, Consumer<Boolean> wrappedValueSetter)
   {
      this.wrappedValueGetter = wrappedValueGetter;
      this.wrappedValueSetter = wrappedValueSetter;
   }

   public void accessImBoolean(Consumer<ImBoolean> imBooleanConsumer)
   {
      // This basic set has no effects to just set it even if the values are the same
      imBoolean.set(wrappedValueGetter.get());
      imBooleanConsumer.accept(imBoolean);
      // wrappedValueSetter might be hooked to a callback, so let's prevent
      // that unless necessary
      boolean imBooleanValue = imBoolean.get();
      if (imBooleanValue != wrappedValueGetter.get())
      {
         wrappedValueSetter.accept(imBooleanValue);
      }
   }
}
