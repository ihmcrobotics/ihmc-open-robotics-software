package us.ihmc.robotDataLogger;

/**
* 
* Topic data type of the struct "LogProperties" defined in "LogProperties.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from LogProperties.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LogProperties.idl instead.
*
*/
public class LogPropertiesPubSubType implements us.ihmc.pubsub.TopicDataType<us.ihmc.robotDataLogger.LogProperties>
{
	public static final java.lang.String name = "us::ihmc::robotDataLogger::LogProperties";
	
	
	
    public LogPropertiesPubSubType()
    {
        
    }

	private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
	private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

    
    @Override
   public void serialize(us.ihmc.robotDataLogger.LogProperties data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }
   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, us.ihmc.robotDataLogger.LogProperties data) throws java.io.IOException
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

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += us.ihmc.robotDataLogger.VariablesPubSubType.getMaxCdrSerializedSize(current_alignment);
	    current_alignment += us.ihmc.robotDataLogger.ModelPubSubType.getMaxCdrSerializedSize(current_alignment);
	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < 255; ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.CameraPubSubType.getMaxCdrSerializedSize(current_alignment);}

	    current_alignment += us.ihmc.robotDataLogger.VideoPubSubType.getMaxCdrSerializedSize(current_alignment);
	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.LogProperties data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.LogProperties data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getVersion().length() + 1;

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getName().length() + 1;

	    current_alignment += us.ihmc.robotDataLogger.VariablesPubSubType.getCdrSerializedSize(data.getVariables(), current_alignment);
	    current_alignment += us.ihmc.robotDataLogger.ModelPubSubType.getCdrSerializedSize(data.getModel(), current_alignment);
	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getTimestamp().length() + 1;

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < data.getCameras().size(); ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.CameraPubSubType.getCdrSerializedSize(data.getCameras().get(a), current_alignment);}

	    current_alignment += us.ihmc.robotDataLogger.VideoPubSubType.getCdrSerializedSize(data.getVideo(), current_alignment);
	
	    return current_alignment - initial_alignment;
	}
	
   public static void write(us.ihmc.robotDataLogger.LogProperties data, us.ihmc.idl.CDR cdr)
   {

	    if(data.getVersion().length() <= 255)
	    cdr.write_type_d(data.getVersion());else
	        throw new RuntimeException("version field exceeds the maximum length");

	    if(data.getName().length() <= 255)
	    cdr.write_type_d(data.getName());else
	        throw new RuntimeException("name field exceeds the maximum length");

	    us.ihmc.robotDataLogger.VariablesPubSubType.write(data.getVariables(), cdr);

	    us.ihmc.robotDataLogger.ModelPubSubType.write(data.getModel(), cdr);

	    if(data.getTimestamp().length() <= 255)
	    cdr.write_type_d(data.getTimestamp());else
	        throw new RuntimeException("timestamp field exceeds the maximum length");

	    if(data.getCameras().size() <= 255)
	    cdr.write_type_e(data.getCameras());else
	        throw new RuntimeException("cameras field exceeds the maximum length");

	    us.ihmc.robotDataLogger.VideoPubSubType.write(data.getVideo(), cdr);
   }

   public static void read(us.ihmc.robotDataLogger.LogProperties data, us.ihmc.idl.CDR cdr)
   {

	    	cdr.read_type_d(data.getVersion());	

	    	cdr.read_type_d(data.getName());	

	    	us.ihmc.robotDataLogger.VariablesPubSubType.read(data.getVariables(), cdr);	

	    	us.ihmc.robotDataLogger.ModelPubSubType.read(data.getModel(), cdr);	

	    	cdr.read_type_d(data.getTimestamp());	

	    	cdr.read_type_e(data.getCameras());	

	    	us.ihmc.robotDataLogger.VideoPubSubType.read(data.getVideo(), cdr);	
   }
   
	@Override
	public final void serialize(us.ihmc.robotDataLogger.LogProperties data, us.ihmc.idl.InterchangeSerializer ser)
	{
			    ser.write_type_d("version", data.getVersion());
			    
			    ser.write_type_d("name", data.getName());
			    
			    ser.write_type_a("variables", new us.ihmc.robotDataLogger.VariablesPubSubType(), data.getVariables());

			    
			    ser.write_type_a("model", new us.ihmc.robotDataLogger.ModelPubSubType(), data.getModel());

			    
			    ser.write_type_d("timestamp", data.getTimestamp());
			    
			    ser.write_type_e("cameras", data.getCameras());
			    
			    ser.write_type_a("video", new us.ihmc.robotDataLogger.VideoPubSubType(), data.getVideo());

			    
	}
	
	@Override
	public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, us.ihmc.robotDataLogger.LogProperties data)
	{
	    			ser.read_type_d("version", data.getVersion());	
	    	    
	    			ser.read_type_d("name", data.getName());	
	    	    
	    			ser.read_type_a("variables", new us.ihmc.robotDataLogger.VariablesPubSubType(), data.getVariables());
	    	
	    	    
	    			ser.read_type_a("model", new us.ihmc.robotDataLogger.ModelPubSubType(), data.getModel());
	    	
	    	    
	    			ser.read_type_d("timestamp", data.getTimestamp());	
	    	    
	    			ser.read_type_e("cameras", data.getCameras());	
	    	    
	    			ser.read_type_a("video", new us.ihmc.robotDataLogger.VideoPubSubType(), data.getVideo());
	    	
	    	    
	}

   public static void staticCopy(us.ihmc.robotDataLogger.LogProperties src, us.ihmc.robotDataLogger.LogProperties dest)
   {
      dest.set(src);
   }
   
   
   @Override
   public us.ihmc.robotDataLogger.LogProperties createData()
   {
      return new us.ihmc.robotDataLogger.LogProperties();
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
   
   public void serialize(us.ihmc.robotDataLogger.LogProperties data, us.ihmc.idl.CDR cdr)
	{
		write(data, cdr);
	}

   public void deserialize(us.ihmc.robotDataLogger.LogProperties data, us.ihmc.idl.CDR cdr)
   {
        read(data, cdr);
   }
   
   public void copy(us.ihmc.robotDataLogger.LogProperties src, us.ihmc.robotDataLogger.LogProperties dest)
   {
      staticCopy(src, dest);
   }	

   
   @Override
   public LogPropertiesPubSubType newInstance()
   {
   	  return new LogPropertiesPubSubType();
   }
}