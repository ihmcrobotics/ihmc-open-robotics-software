package us.ihmc.robotDataLogger;

/**
* 
* Topic data type of the struct "LoggerConfiguration" defined in "LoggerConfiguration.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from LoggerConfiguration.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LoggerConfiguration.idl instead.
*
*/
public class LoggerConfigurationPubSubType implements us.ihmc.pubsub.TopicDataType<us.ihmc.robotDataLogger.LoggerConfiguration>
{
	public static final java.lang.String name = "us::ihmc::robotDataLogger::LoggerConfiguration";
	
	
	
    public LoggerConfigurationPubSubType()
    {
        
    }

	private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
	private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

    
    @Override
   public void serialize(us.ihmc.robotDataLogger.LoggerConfiguration data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }
   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, us.ihmc.robotDataLogger.LoggerConfiguration data) throws java.io.IOException
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

	    current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.LoggerConfiguration data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.LoggerConfiguration data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getCamerasToCapture().length() + 1;

	    current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

	
	    return current_alignment - initial_alignment;
	}
	
   public static void write(us.ihmc.robotDataLogger.LoggerConfiguration data, us.ihmc.idl.CDR cdr)
   {

	    if(data.getCamerasToCapture().length() <= 255)
	    cdr.write_type_d(data.getCamerasToCapture());else
	        throw new RuntimeException("camerasToCapture field exceeds the maximum length");

	    cdr.write_type_7(data.getPublicBroadcast());
   }

   public static void read(us.ihmc.robotDataLogger.LoggerConfiguration data, us.ihmc.idl.CDR cdr)
   {

	    	cdr.read_type_d(data.getCamerasToCapture());	

	    	data.setPublicBroadcast(cdr.read_type_7());
	    	
   }
   
	@Override
	public final void serialize(us.ihmc.robotDataLogger.LoggerConfiguration data, us.ihmc.idl.InterchangeSerializer ser)
	{
			    ser.write_type_d("camerasToCapture", data.getCamerasToCapture());
			    
			    ser.write_type_7("publicBroadcast", data.getPublicBroadcast());
			    
	}
	
	@Override
	public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, us.ihmc.robotDataLogger.LoggerConfiguration data)
	{
	    			ser.read_type_d("camerasToCapture", data.getCamerasToCapture());	
	    	    
	    			data.setPublicBroadcast(ser.read_type_7("publicBroadcast"));	
	    	    
	}

   public static void staticCopy(us.ihmc.robotDataLogger.LoggerConfiguration src, us.ihmc.robotDataLogger.LoggerConfiguration dest)
   {
      dest.set(src);
   }
   
   
   @Override
   public us.ihmc.robotDataLogger.LoggerConfiguration createData()
   {
      return new us.ihmc.robotDataLogger.LoggerConfiguration();
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
   
   public void serialize(us.ihmc.robotDataLogger.LoggerConfiguration data, us.ihmc.idl.CDR cdr)
	{
		write(data, cdr);
	}

   public void deserialize(us.ihmc.robotDataLogger.LoggerConfiguration data, us.ihmc.idl.CDR cdr)
   {
        read(data, cdr);
   }
   
   public void copy(us.ihmc.robotDataLogger.LoggerConfiguration src, us.ihmc.robotDataLogger.LoggerConfiguration dest)
   {
      staticCopy(src, dest);
   }	

   
   @Override
   public LoggerConfigurationPubSubType newInstance()
   {
   	  return new LoggerConfigurationPubSubType();
   }
}