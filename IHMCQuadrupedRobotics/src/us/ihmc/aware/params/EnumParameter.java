package us.ihmc.aware.params;

public class EnumParameter<E extends Enum<E>> extends Parameter
{
   private final Class<E> enumType;
   private E value;

   EnumParameter(String path, Class<E> enumType, E defaultValue)
   {
      super(path);
      this.enumType = enumType;
      this.value = defaultValue;
   }

   public E get()
   {
      return value;
   }

   public void set(E value)
   {
      this.value = value;
      notifyChangedListeners();
   }

   @Override
   public boolean tryLoadValue(String value)
   {
      this.value = E.valueOf(enumType, value);
      return true;
   }

   @Override
   String dumpValue()
   {
      return value.toString();
   }
}
