package us.ihmc.robotDataLogger;

/**
* 
* Definition of the enum "HandshakeFileType" defined in LogProperties.idl. 
*
* This file was automatically generated from LogProperties.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LogProperties.idl instead.
*
*/
import us.ihmc.idl.IDLTools;

public enum HandshakeFileType
{
         PROTOBUFFER,
      
         IDL_YAML,
      
         IDL_CDR,
      
   ;
   public static HandshakeFileType[] values = values();

   public boolean epsilonEquals(HandshakeFileType other, double epsilon)
   {
      return IDLTools.epsilonEqualsEnum(this, other, epsilon);
   }
}