package us.ihmc.robotDataLogger;

/**
* 
* Topic data type of the struct "ClearLogRequest" defined in "ClearLogRequest.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ClearLogRequest.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ClearLogRequest.idl instead.
*
*/
public class ClearLogRequestPubSubType implements us.ihmc.pubsub.TopicDataType<us.ihmc.robotDataLogger.ClearLogRequest>
{
	public static final java.lang.String name = "us::ihmc::robotDataLogger::ClearLogRequest";
	
	
	
    public ClearLogRequestPubSubType()
    {
        
    }

	private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
	private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

    
    @Override
   public void serialize(us.ihmc.robotDataLogger.ClearLogRequest data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }
   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, us.ihmc.robotDataLogger.ClearLogRequest data) throws java.io.IOException
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
	            
	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.ClearLogRequest data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.ClearLogRequest data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getGuid().length() + 1;
	
	    return current_alignment - initial_alignment;
	}
	
   public static void write(us.ihmc.robotDataLogger.ClearLogRequest data, us.ihmc.idl.CDR cdr)
   {

	    if(data.getGuid().length() <= 255)
	    cdr.write_type_d(data.getGuid());else
	        throw new RuntimeException("guid field exceeds the maximum length");
   }

   public static void read(us.ihmc.robotDataLogger.ClearLogRequest data, us.ihmc.idl.CDR cdr)
   {

	    	cdr.read_type_d(data.getGuid());	
   }
   
	@Override
	public final void serialize(us.ihmc.robotDataLogger.ClearLogRequest data, us.ihmc.idl.InterchangeSerializer ser)
	{
			    ser.write_type_d("guid", data.getGuid());
			    
	}
	
	@Override
	public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, us.ihmc.robotDataLogger.ClearLogRequest data)
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
      return us.ihmc.idl.CDR.getTypeSize(getMaxCdrSerializedSize());
   }

   @Override
   public java.lang.String getName()
   {
      return name;
   }
   
   public void serialize(us.ihmc.robotDataLogger.ClearLogRequest data, us.ihmc.idl.CDR cdr)
	{
		write(data, cdr);
	}

   public void deserialize(us.ihmc.robotDataLogger.ClearLogRequest data, us.ihmc.idl.CDR cdr)
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