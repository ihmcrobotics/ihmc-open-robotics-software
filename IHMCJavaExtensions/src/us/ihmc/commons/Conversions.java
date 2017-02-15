package us.ihmc.commons;

public class Conversions
{
   // Bytes
   public static final int KIBIBYTES_TO_BYTES = 1024;
   public static final int KILOBYTES_TO_BYTES = 1000;

   public static int kibibytesToBytes(int kibibytes)
   {
      return kibibytes * KIBIBYTES_TO_BYTES;
   }

   public static int kilobytesToBytes(int kilobytes)
   {
      return kilobytes * KILOBYTES_TO_BYTES;
   }

   public static int mebibytesToBytes(int mebibytes)
   {
      return mebibytes * (int) Math.pow(KIBIBYTES_TO_BYTES, 2);
   }

   public static int megabytesToBytes(int megabytes)
   {
      return megabytes * (int) Math.pow(KILOBYTES_TO_BYTES, 2);
   }
}
