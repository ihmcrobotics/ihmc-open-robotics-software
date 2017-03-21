package us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.CDR;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.idl.IDLStruct;
import java.util.Arrays;

/**
* 
* Definition of the enum "YoType" defined in Handshake.idl. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
public enum YoType implements IDLStruct<YoType>
{
        	DoubleYoVariable,
        
        	BooleanYoVariable,
        
        	IntegerYoVariable,
        
        	LongYoVariable,
        
        	EnumYoVariable,
        
	;
	
	public static YoType[] values = values();

}