package us.ihmc.robotDataLogger;

/**
* 
* Definition of the enum "JointType" defined in Handshake.idl. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
import us.ihmc.idl.IDLTools;

public enum JointType
{
         SiXDoFJoint,
      
         OneDoFJoint,
      
   ;
   public static JointType[] values = values();

   public boolean epsilonEquals(JointType other, double epsilon)
   {
      return IDLTools.epsilonEqualsEnum(this, other, epsilon);
   }
}