package us.ihmc.utilities.ros.msgToPacket.converter;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class RosEnumConversionException extends Exception
{
   public RosEnumConversionException(Class<? extends Enum> enumClass, byte requestedOrdinal, String message)
   {
      super("Conversion error! Could not convert ordinal of value " + requestedOrdinal + " to Enum of type " + enumClass.getSimpleName() + System.lineSeparator() + message);
   }
}
