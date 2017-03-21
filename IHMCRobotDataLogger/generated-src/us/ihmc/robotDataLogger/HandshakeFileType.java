package us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.CDR;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.idl.IDLStruct;
import java.util.Arrays;

/**
* 
* Definition of the enum "HandshakeFileType" defined in LogProperties.idl. 
*
* This file was automatically generated from LogProperties.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LogProperties.idl instead.
*
*/
public enum HandshakeFileType implements IDLStruct<HandshakeFileType>
{
        	PROTOBUFFER,
        
        	IDL_YAML,
        
	;
	
	public static HandshakeFileType[] values = values();

}