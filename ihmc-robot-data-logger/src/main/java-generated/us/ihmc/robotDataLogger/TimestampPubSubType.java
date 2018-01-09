package us.ihmc.robotDataLogger;

/**
* 
* Topic data type of the struct "Timestamp" defined in "Timestamp.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Timestamp.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Timestamp.idl instead.
*
*/
public class TimestampPubSubType implements us.ihmc.pubsub.TopicDataType<us.ihmc.robotDataLogger.Timestamp>
{
	public static final java.lang.String name = "us::ihmc::robotDataLogger::Timestamp";
	
	
	
    public TimestampPubSubType()
    {
        
    }

	private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
	private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

    
    @Override
   public void serialize(us.ihmc.robotDataLogger.Timestamp data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }
   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, us.ihmc.robotDataLogger.Timestamp data) throws java.io.IOException
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
	            
	    current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.Timestamp data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.Timestamp data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

	
	    return current_alignment - initial_alignment;
	}
	
   public static void write(us.ihmc.robotDataLogger.Timestamp data, us.ihmc.idl.CDR cdr)
   {

	    cdr.write_type_11(data.getTimestamp());
   }

   public static void read(us.ihmc.robotDataLogger.Timestamp data, us.ihmc.idl.CDR cdr)
   {

	    	data.setTimestamp(cdr.read_type_11());
	    	
   }
   
	@Override
	public final void serialize(us.ihmc.robotDataLogger.Timestamp data, us.ihmc.idl.InterchangeSerializer ser)
	{
			    ser.write_type_11("timestamp", data.getTimestamp());
			    
	}
	
	@Override
	public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, us.ihmc.robotDataLogger.Timestamp data)
	{
	    			data.setTimestamp(ser.read_type_11("timestamp"));	
	    	    
	}

   public static void staticCopy(us.ihmc.robotDataLogger.Timestamp src, us.ihmc.robotDataLogger.Timestamp dest)
   {
      dest.set(src);
   }
   
   
   @Override
   public us.ihmc.robotDataLogger.Timestamp createData()
   {
      return new us.ihmc.robotDataLogger.Timestamp();
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
   
   public void serialize(us.ihmc.robotDataLogger.Timestamp data, us.ihmc.idl.CDR cdr)
	{
		write(data, cdr);
	}

   public void deserialize(us.ihmc.robotDataLogger.Timestamp data, us.ihmc.idl.CDR cdr)
   {
        read(data, cdr);
   }
   
   public void copy(us.ihmc.robotDataLogger.Timestamp src, us.ihmc.robotDataLogger.Timestamp dest)
   {
      staticCopy(src, dest);
   }	

   
   @Override
   public TimestampPubSubType newInstance()
   {
   	  return new TimestampPubSubType();
   }
}