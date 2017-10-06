package us.ihmc.tools.containers;

public class EnumTools
{
   private EnumTools()
   {
      // Prevent initialization
   }
   
   public static <T extends Enum<T>> T getNext(T current)
   {
      T[] enumConstants = current.getDeclaringClass().getEnumConstants();
      int currentOrdinal = current.ordinal();
      int nextOrdinal = (currentOrdinal + 1) % enumConstants.length;
      return enumConstants[nextOrdinal];
   }
}
