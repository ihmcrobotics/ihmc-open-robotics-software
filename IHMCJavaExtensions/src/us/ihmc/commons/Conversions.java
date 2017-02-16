package us.ihmc.commons;

/**
 * Common conversions useful for making use of some libraries easier to read.
 */
public class Conversions
{
   /** Number of bytes (B) in a kibibyte (KiB) */
   public static final int KIBIBYTES_TO_BYTES = 1024;
   
   /** Number of bytes (B) in a kilobyte (KB) */
   public static final int KILOBYTES_TO_BYTES = 1000;

   private Conversions()
   {
      // Disallow construction
   }
   
   /**
    * Convert kibibytes (KiB) to bytes (B).
    * 
    * @param kibibytes
    * @return bytes
    */
   public static int kibibytesToBytes(int kibibytes)
   {
      return kibibytes * KIBIBYTES_TO_BYTES;
   }

   /**
    * Convert kilobytes (KB) to bytes (B).
    * 
    * @param kilobytes
    * @return bytes
    */
   public static int kilobytesToBytes(int kilobytes)
   {
      return kilobytes * KILOBYTES_TO_BYTES;
   }

   /**
    * Convert mebibytes (MiB) to bytes (B).
    * 
    * @param mebibytes
    * @return bytes
    */
   public static int mebibytesToBytes(int mebibytes)
   {
      return mebibytes * (int) Math.pow(KIBIBYTES_TO_BYTES, 2);
   }

   /**
    * Convert megabytes (MB) to bytes (B).
    * 
    * @param megabytes
    * @return bytes
    */
   public static int megabytesToBytes(int megabytes)
   {
      return megabytes * (int) Math.pow(KILOBYTES_TO_BYTES, 2);
   }
}
