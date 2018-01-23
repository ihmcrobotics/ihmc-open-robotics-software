package us.ihmc.robotDataLogger;

/**
* 
* Topic data type of the struct "LogData" defined in "LogData.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from LogData.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LogData.idl instead.
*
*/
public class LogDataPubSubType implements us.ihmc.pubsub.TopicDataType<us.ihmc.robotDataLogger.LogData>
{
	public static final java.lang.String name = "us::ihmc::robotDataLogger::LogData";
	
	
	
    public LogDataPubSubType()
    {
        
    }

	private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
	private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

    
    @Override
   public void serialize(us.ihmc.robotDataLogger.LogData data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }
   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, us.ihmc.robotDataLogger.LogData data) throws java.io.IOException
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

	    current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

	    current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
	    current_alignment += (100 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
	    current_alignment += (100 * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);


	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.LogData data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.LogData data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

	    current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

	    current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
	    current_alignment += (data.getData().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
	    current_alignment += (data.getJointStates().size() * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);


	
	    return current_alignment - initial_alignment;
	}
	
   public static void write(us.ihmc.robotDataLogger.LogData data, us.ihmc.idl.CDR cdr)
   {

	    cdr.write_type_11(data.getUid());

	    cdr.write_type_11(data.getTimestamp());

	    cdr.write_type_11(data.getTransmitTime());

	    cdr.write_type_c(data.getType().ordinal());


	    cdr.write_type_2(data.getRegistry());

	    cdr.write_type_2(data.getOffset());

	    cdr.write_type_2(data.getNumberOfVariables());

	    if(data.getData().size() <= 100)
	    cdr.write_type_e(data.getData());else
	        throw new RuntimeException("data field exceeds the maximum length");

	    if(data.getJointStates().size() <= 100)
	    cdr.write_type_e(data.getJointStates());else
	        throw new RuntimeException("jointStates field exceeds the maximum length");
   }

   public static void read(us.ihmc.robotDataLogger.LogData data, us.ihmc.idl.CDR cdr)
   {

	    	data.setUid(cdr.read_type_11());
	    	

	    	data.setTimestamp(cdr.read_type_11());
	    	

	    	data.setTransmitTime(cdr.read_type_11());
	    	

	    	data.setType(us.ihmc.robotDataLogger.LogDataType.values[cdr.read_type_c()]);
	    	

	    	data.setRegistry(cdr.read_type_2());
	    	

	    	data.setOffset(cdr.read_type_2());
	    	

	    	data.setNumberOfVariables(cdr.read_type_2());
	    	

	    	cdr.read_type_e(data.getData());	

	    	cdr.read_type_e(data.getJointStates());	
   }
   
	@Override
	public final void serialize(us.ihmc.robotDataLogger.LogData data, us.ihmc.idl.InterchangeSerializer ser)
	{
			    ser.write_type_11("uid", data.getUid());
			    
			    ser.write_type_11("timestamp", data.getTimestamp());
			    
			    ser.write_type_11("transmitTime", data.getTransmitTime());
			    
			    ser.write_type_c("type", data.getType());
			    
			    ser.write_type_2("registry", data.getRegistry());
			    
			    ser.write_type_2("offset", data.getOffset());
			    
			    ser.write_type_2("numberOfVariables", data.getNumberOfVariables());
			    
			    ser.write_type_e("data", data.getData());
			    
			    ser.write_type_e("jointStates", data.getJointStates());
			    
	}
	
	@Override
	public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, us.ihmc.robotDataLogger.LogData data)
	{
	    			data.setUid(ser.read_type_11("uid"));	
	    	    
	    			data.setTimestamp(ser.read_type_11("timestamp"));	
	    	    
	    			data.setTransmitTime(ser.read_type_11("transmitTime"));	
	    	    
	    			data.setType((us.ihmc.robotDataLogger.LogDataType)ser.read_type_c("type", us.ihmc.robotDataLogger.LogDataType.class));
	    	
	    	    
	    			data.setRegistry(ser.read_type_2("registry"));	
	    	    
	    			data.setOffset(ser.read_type_2("offset"));	
	    	    
	    			data.setNumberOfVariables(ser.read_type_2("numberOfVariables"));	
	    	    
	    			ser.read_type_e("data", data.getData());	
	    	    
	    			ser.read_type_e("jointStates", data.getJointStates());	
	    	    
	}

   public static void staticCopy(us.ihmc.robotDataLogger.LogData src, us.ihmc.robotDataLogger.LogData dest)
   {
      dest.set(src);
   }
   
   
   @Override
   public us.ihmc.robotDataLogger.LogData createData()
   {
      return new us.ihmc.robotDataLogger.LogData();
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
   
   public void serialize(us.ihmc.robotDataLogger.LogData data, us.ihmc.idl.CDR cdr)
	{
		write(data, cdr);
	}

   public void deserialize(us.ihmc.robotDataLogger.LogData data, us.ihmc.idl.CDR cdr)
   {
        read(data, cdr);
   }
   
   public void copy(us.ihmc.robotDataLogger.LogData src, us.ihmc.robotDataLogger.LogData dest)
   {
      staticCopy(src, dest);
   }	

   
   @Override
   public LogDataPubSubType newInstance()
   {
   	  return new LogDataPubSubType();
   }
}