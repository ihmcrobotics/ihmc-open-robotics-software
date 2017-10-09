package us.ihmc.robotDataLogger;

import java.io.IOException;

import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.common.SerializedPayload;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.idl.CDR;
import us.ihmc.idl.IDLSequence;

/**
* 
* Topic data type of the struct "ClearLogRequest" defined in "ClearLogRequest.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ClearLogRequest.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ClearLogRequest.idl instead.
*
*/
public class ClearLogRequestPubSubType implements TopicDataType<us.ihmc.robotDataLogger.ClearLogRequest>
{
	public static final String name = "us::ihmc::robotDataLogger::ClearLogRequest";
	
	
	
    public ClearLogRequestPubSubType()
    {
        
    }

	private final CDR serializeCDR = new CDR();
	private final CDR deserializeCDR = new CDR();

    
    @Override
   public void serialize(us.ihmc.robotDataLogger.ClearLogRequest data, SerializedPayload serializedPayload) throws IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }
   @Override
   public void deserialize(SerializedPayload serializedPayload, us.ihmc.robotDataLogger.ClearLogRequest data) throws IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }
   
	public static int getMaxCdrSerializedSize()
	{
		return getMaxCdrSerializedSize(0);
	}

	public static int getMaxCdrSerializedSize(int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;
	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.ClearLogRequest data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.ClearLogRequest data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getGuid().length() + 1;
	
	    return current_alignment - initial_alignment;
	}
	
   public static void write(us.ihmc.robotDataLogger.ClearLogRequest data, CDR cdr)
   {

	    if(data.getGuid().length() <= 255)
	    cdr.write_type_d(data.getGuid());else
	        throw new RuntimeException("guid field exceeds the maximum length");
   }

   public static void read(us.ihmc.robotDataLogger.ClearLogRequest data, CDR cdr)
   {

	    	cdr.read_type_d(data.getGuid());	
   }
   
	@Override
	public final void serialize(us.ihmc.robotDataLogger.ClearLogRequest data, InterchangeSerializer ser)
	{
			    ser.write_type_d("guid", data.getGuid());
			    
	}
	
	@Override
	public final void deserialize(InterchangeSerializer ser, us.ihmc.robotDataLogger.ClearLogRequest data)
	{
	    			ser.read_type_d("guid", data.getGuid());	
	    	    
	}

   public static void staticCopy(us.ihmc.robotDataLogger.ClearLogRequest src, us.ihmc.robotDataLogger.ClearLogRequest dest)
   {
      dest.set(src);
   }
   
   
   @Override
   public us.ihmc.robotDataLogger.ClearLogRequest createData()
   {
      return new us.ihmc.robotDataLogger.ClearLogRequest();
   }
      

   @Override
   public int getTypeSize()
   {
      return CDR.getTypeSize(getMaxCdrSerializedSize());
   }

   @Override
   public String getName()
   {
      return name;
   }
   
   public void serialize(us.ihmc.robotDataLogger.ClearLogRequest data, CDR cdr)
	{
		write(data, cdr);
	}

   public void deserialize(us.ihmc.robotDataLogger.ClearLogRequest data, CDR cdr)
   {
        read(data, cdr);
   }
   
   public void copy(us.ihmc.robotDataLogger.ClearLogRequest src, us.ihmc.robotDataLogger.ClearLogRequest dest)
   {
      staticCopy(src, dest);
   }	

   
   @Override
   public ClearLogRequestPubSubType newInstance()
   {
   	  return new ClearLogRequestPubSubType();
   }
}