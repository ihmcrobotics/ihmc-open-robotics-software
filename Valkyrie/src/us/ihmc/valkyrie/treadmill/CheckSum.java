package us.ihmc.valkyrie.treadmill;

/**
* X.25 CRC calculation for MAVlink messages.
* The checksum must be initialized, 
* updated with witch field of the message,
* and then finished with the message id.
*
*/
public class CheckSum
{
   /**
    *
    *  CALCULATE THE CHECKSUM
    *
    */

   static final char X25_INIT_CRC = 0xffff;
   static final char X25_VALIDATE_CRC = 0xf0b8;

   /**
    * @brief Accumulate the X.25 CRC by adding one char at a time.
    *
    * The checksum function adds the hash of one char at a time to the
    * 16 bit checksum (uint16_t).
    *
    * @param data new char to hash
    * @param crcAccum the already accumulated checksum
    **/
   public static char crc_accumulate(byte data, char crcAccum)
   {

      /* Accumulate one byte of data into the CRC */
      byte tmp;
      tmp = (byte) (data ^ (byte) (crcAccum & 0xff));
      tmp ^= (tmp << 4);

      char tmp1 = (char) (((char) tmp) & 0x00FF);
      crcAccum = (char) ((crcAccum >> 8) ^ (tmp1 << 8) ^ (tmp1 << 3) ^ (tmp1 >> 4));
      return crcAccum;
   }

   /**
    * @brief Initiliaze the buffer for the X.25 CRC
    *
    * @param crcAccum the 16 bit X.25 CRC
    */
   public static char crc_init()
   {
      return X25_INIT_CRC;
   }

   /**
    * @brief Calculates the X.25 checksum on an Array of Bytes
    *
    * @param  pBuffer array containing the bytes to hash
    * @return the checksum over the array of bytes
    **/
   public static int crc_calculate(byte pBuffer[])
   {
      char crcTmp = crc_init();
      for (byte b : pBuffer)
      {
         crcTmp = crc_accumulate(b, crcTmp);
      }
      return crcTmp;
   }
}