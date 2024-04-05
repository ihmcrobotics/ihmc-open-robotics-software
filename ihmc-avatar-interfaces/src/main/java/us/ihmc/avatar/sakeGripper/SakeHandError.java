package us.ihmc.avatar.sakeGripper;

/**
 * Errors to be sent over EtherCAT from Arduino. Needs to match Arduino error indices.
 */
public enum SakeHandErrorCode
{
   // Values based off of Dynamixel Protocol 2.0 Error codes
   NONE(0b00000000),
   OVERLOAD_ERROR(0b00100000),
   ELECTRICAL_SHOCK_ERROR(0b00010000),
   MOTOR_ENCODER_ERROR(0b00001000),
   OVER_HEATING_ERROR(0b00000100),
   INPUT_VOLTAGE_ERROR(0b00000001);

   public final int errorCode;

   SakeHandErrorCode(int errorCode)
   {
      this.errorCode = errorCode;
   }
}
