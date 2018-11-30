package us.ihmc.robotDataLogger;

/**
* 
* Definition of the enum "LoadStatus" defined in Handshake.idl. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
import us.ihmc.idl.IDLTools;

public enum LoadStatus
{
         NoParameter,
      
         Unloaded,
      
         Default,
      
         Loaded,
      
   ;
   public static LoadStatus[] values = values();

   public boolean epsilonEquals(LoadStatus other, double epsilon)
   {
      return IDLTools.epsilonEqualsEnum(this, other, epsilon);
   }
}