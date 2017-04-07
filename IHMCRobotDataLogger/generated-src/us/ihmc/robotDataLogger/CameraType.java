package us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.CDR;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.idl.IDLStruct;
import java.util.Arrays;

/**
* 
* Definition of the enum "CameraType" defined in Announcement.idl. 
*
* This file was automatically generated from Announcement.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Announcement.idl instead.
*
*/
public enum CameraType implements IDLStruct<CameraType>
{
        	CAPTURE_CARD,
        
        	NETWORK_STREAM,
        
	;
	
	public static CameraType[] values = values();

}