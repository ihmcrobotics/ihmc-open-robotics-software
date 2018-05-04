package us.ihmc.robotDataLogger;

/**
* 
* Definition of the enum "YoType" defined in Handshake.idl. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
import us.ihmc.idl.IDLTools;

public enum YoType
{
         DoubleYoVariable,
      
         BooleanYoVariable,
      
         IntegerYoVariable,
      
         LongYoVariable,
      
         EnumYoVariable,
      
   ;
   public static YoType[] values = values();

   public boolean epsilonEquals(YoType other, double epsilon)
   {
      return IDLTools.epsilonEqualsEnum(this, other, epsilon);
   }
}