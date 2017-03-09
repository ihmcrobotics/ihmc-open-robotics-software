package us.ihmc.robotics.nameBasedHashCode;

public class NameBasedHashCodeTools
{
   public static final long DEFAULT_HASHCODE = 1L;
   public static final long NULL_HASHCODE = 0L;
   public static final long PRIME = 31L;

   public static long computeArrayHashCode(NameBasedHashCodeHolder holders[])
   {
      if (holders == null)
         return NULL_HASHCODE;

      long nameBasedHashCode = DEFAULT_HASHCODE;

      for (int i = 0; i < holders.length; i++)
         nameBasedHashCode = combineHashCodes(nameBasedHashCode, holders[i]);

      return nameBasedHashCode;
   }

   public static long computeSubArrayHashCode(NameBasedHashCodeHolder holders[], int firstIndex, int lastIndex)
   {
      if (holders == null)
         return NULL_HASHCODE;

      long nameBasedHashCode = DEFAULT_HASHCODE;

      for (int i = firstIndex; i <= lastIndex; i++)
         nameBasedHashCode = combineHashCodes(nameBasedHashCode, holders[i]);

      return nameBasedHashCode;
   }

   public static long computeStringHashCode(String string)
   {
      if (string == null)
         return NULL_HASHCODE;

      long h = DEFAULT_HASHCODE;

      for (int i = 0; i < string.length(); i++)
         h = PRIME * h + string.charAt(i);

      return h;
   }

   public static long combineHashCodes(long hashCodeToUpdate, NameBasedHashCodeHolder holder)
   {
      return PRIME * hashCodeToUpdate + (holder == null ? NULL_HASHCODE : holder.getNameBasedHashCode());
   }

   public static long combineHashCodes(NameBasedHashCodeHolder holder1, NameBasedHashCodeHolder holder2)
   {
      return PRIME * (holder1 == null ? NULL_HASHCODE : holder1.getNameBasedHashCode()) + (holder2 == null ? NULL_HASHCODE : holder2.getNameBasedHashCode());
   }

   public static long combineHashCodes(String string, NameBasedHashCodeHolder holder)
   {
      return PRIME * computeStringHashCode(string) + (holder == null ? NULL_HASHCODE : holder.getNameBasedHashCode());
   }
}
