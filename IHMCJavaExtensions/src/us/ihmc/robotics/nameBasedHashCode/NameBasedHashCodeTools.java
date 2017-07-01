package us.ihmc.robotics.nameBasedHashCode;

/**
 * {@code NameBasedHashCodeHolder} provides a series of tools to calculate {@code String} based
 * unique hash codes in addition to some default values.
 * <p>
 * Use these tools with {@code NameBasedHashCodeHolder} to ensure name based hash codes are
 * calculated using the same algorithm.
 * </p>
 */
public class NameBasedHashCodeTools
{
   /** Smallest prime number used as the base for computing name based hash codes. */
   public static final long DEFAULT_HASHCODE = 1L;
   /** Default hash code for any object that is equal to {@code null}. */
   public static final long NULL_HASHCODE = 0L;
   /** Prime number that is used when combining two hash codes into one. */
   public static final long PRIME = 31L;

   /**
    * Combines the hash codes from each of the {@code holders} to calculate a new name based hash
    * code that represents the given array.
    * 
    * @param holders the objects which hash codes are to be combined. Not modified.
    * @return the combined unique hash code.
    */
   public static long computeArrayHashCode(NameBasedHashCodeHolder holders[])
   {
      if (holders == null)
         return NULL_HASHCODE;

      long nameBasedHashCode = DEFAULT_HASHCODE;

      for (int i = 0; i < holders.length; i++)
         nameBasedHashCode = combineHashCodes(nameBasedHashCode, holders[i]);

      return nameBasedHashCode;
   }

   /**
    * Combines the hash codes from each of the {@code holders} &in; [ {@code firstIndex},
    * {@code secondIndex} ] to calculate a new name based hash code that represents the sub-array.
    * 
    * @param holders the objects which hash codes are to be combined. Not modified.
    * @param firstIndex index of the first object to use for computing the combined hash code.
    * @param lastIndex index of the last object to use for computing the combined hash code.
    * @return the combined unique hash code.
    */
   public static long computeSubArrayHashCode(NameBasedHashCodeHolder holders[], int firstIndex, int lastIndex)
   {
      if (holders == null)
         return NULL_HASHCODE;

      long nameBasedHashCode = DEFAULT_HASHCODE;

      for (int i = firstIndex; i <= lastIndex; i++)
         nameBasedHashCode = combineHashCodes(nameBasedHashCode, holders[i]);

      return nameBasedHashCode;
   }

   /**
    * Computes the unique hash code from the given {@code string}.
    * <p>
    * The hash code is computed using {@code String#hashCode()} when the object is not {@code null}.
    * If the object is {@code null}, this method returns {@value #NULL_HASHCODE}.
    * </p>
    * 
    * @param string the string to compute the hash code of.
    * @return the string hash code.
    */
   public static long computeStringHashCode(String string)
   {
      if (string == null)
         return NULL_HASHCODE;

      return string.hashCode();
   }

   /**
    * Combines two hash codes into one.
    * 
    * @param hashCodeToUpdate the value of the first hash code to be combined.
    * @param holder the object which hash code is to be combined. Not modified.
    * @return the value of the combined hash code.
    */
   public static long combineHashCodes(long hashCodeToUpdate, NameBasedHashCodeHolder holder)
   {
      return PRIME * hashCodeToUpdate + (holder == null ? NULL_HASHCODE : holder.getNameBasedHashCode());
   }

   /**
    * Combines two hash codes into one.
    * 
    * @param holder1 the first object which hash code is to be combined. Not modified.
    * @param holder2 the second object which hash code is to be combined. Not modified.
    * @return the value of the combined hash code.
    */
   public static long combineHashCodes(NameBasedHashCodeHolder holder1, NameBasedHashCodeHolder holder2)
   {
      return PRIME * (holder1 == null ? NULL_HASHCODE : holder1.getNameBasedHashCode()) + (holder2 == null ? NULL_HASHCODE : holder2.getNameBasedHashCode());
   }

   /**
    * Combines two hash codes into one.
    *
    * @param string string from which the hash code is to be computed and then combined.
    * @param holder the object which hash code is to be combined. Not modified.
    * @return the value of the combined hash code.
    */
   public static long combineHashCodes(String string, NameBasedHashCodeHolder holder)
   {
      return PRIME * computeStringHashCode(string) + (holder == null ? NULL_HASHCODE : holder.getNameBasedHashCode());
   }

   /**
    * Combines two hash codes into one.
    *
    * @param string1 first string from which the hash code is to be computed and then combined.
    * @param string2 second string from which the hash code is to be computed and then combined.
    * @return the value of the combined hash code.
    */
   public static long combineHashCodes(String string1, String string2)
   {
      return PRIME * computeStringHashCode(string1) + computeStringHashCode(string2);
   }
}
