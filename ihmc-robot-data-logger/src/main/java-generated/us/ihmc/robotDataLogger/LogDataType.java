package us.ihmc.robotDataLogger;

/**
* 
* Definition of the enum "LogDataType" defined in LogData.idl. 
*
* This file was automatically generated from LogData.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LogData.idl instead.
*
*/
import us.ihmc.idl.IDLTools;

public enum LogDataType
{
         KEEP_ALIVE_PACKET,
      
         DATA_PACKET,
      
         VIDEO_PACKET,
      
   ;
   public static LogDataType[] values = values();

   public boolean epsilonEquals(LogDataType other, double epsilon)
   {
      return IDLTools.epsilonEqualsEnum(this, other, epsilon);
   }
}