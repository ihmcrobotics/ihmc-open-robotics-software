package us.ihmc.avatar.sakeGripper;

/**
 * Errors to be sent from hand's Dynamixel over EtherCAT.
 */
public enum SakeHandError
{
   // Values based off of Dynamixel Protocol 1.0 Error codes (https://emanual.robotis.com/docs/en/dxl/protocol1/#error)
   NONE                 (0b00000000),
   OVERLOAD_ERROR       (0b00100000),
   CHECKSUM_ERROR       (0b00010000),
   RANGE_ERROR          (0b00001000),
   OVER_HEATING_ERROR   (0b00000100),
   ANGLE_LIMIT_ERROR    (0b00000010),
   INPUT_VOLTAGE_ERROR  (0b00000001);

   public final int errorCode;

   SakeHandError(int errorCode)
   {
      this.errorCode = errorCode;
   }

   @Override
   public String toString()
   {
      return name().replace("_", " ");
   }
}
