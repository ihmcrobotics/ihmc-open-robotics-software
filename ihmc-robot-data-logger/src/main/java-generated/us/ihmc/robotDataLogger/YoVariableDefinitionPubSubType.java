package us.ihmc.robotDataLogger;

/**
* 
* Topic data type of the struct "YoVariableDefinition" defined in "Handshake.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
public class YoVariableDefinitionPubSubType implements us.ihmc.pubsub.TopicDataType<us.ihmc.robotDataLogger.YoVariableDefinition>
{
	public static final java.lang.String name = "us::ihmc::robotDataLogger::YoVariableDefinition";
	
	
	
    public YoVariableDefinitionPubSubType()
    {
        
    }

	private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
	private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

    
    @Override
   public void serialize(us.ihmc.robotDataLogger.YoVariableDefinition data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }
   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, us.ihmc.robotDataLogger.YoVariableDefinition data) throws java.io.IOException
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

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

	    current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

	    current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

	    current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

	    current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

	    current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

	    current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.YoVariableDefinition data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.YoVariableDefinition data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getName().length() + 1;

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getDescription().length() + 1;

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

	    current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

	    current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

	    current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

	    current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

	    current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

	    current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

	
	    return current_alignment - initial_alignment;
	}
	
   public static void write(us.ihmc.robotDataLogger.YoVariableDefinition data, us.ihmc.idl.CDR cdr)
   {

	    if(data.getName().length() <= 255)
	    cdr.write_type_d(data.getName());else
	        throw new RuntimeException("name field exceeds the maximum length");

	    if(data.getDescription().length() <= 255)
	    cdr.write_type_d(data.getDescription());else
	        throw new RuntimeException("description field exceeds the maximum length");

	    cdr.write_type_c(data.getType().ordinal());


	    cdr.write_type_3(data.getRegistry());

	    cdr.write_type_3(data.getEnumType());

	    cdr.write_type_7(data.getAllowNullValues());

	    cdr.write_type_7(data.getIsParameter());

	    cdr.write_type_6(data.getMin());

	    cdr.write_type_6(data.getMax());

	    cdr.write_type_c(data.getLoadStatus().ordinal());

   }

   public static void read(us.ihmc.robotDataLogger.YoVariableDefinition data, us.ihmc.idl.CDR cdr)
   {

	    	cdr.read_type_d(data.getName());	

	    	cdr.read_type_d(data.getDescription());	

	    	data.setType(us.ihmc.robotDataLogger.YoType.values[cdr.read_type_c()]);
	    	

	    	data.setRegistry(cdr.read_type_3());
	    	

	    	data.setEnumType(cdr.read_type_3());
	    	

	    	data.setAllowNullValues(cdr.read_type_7());
	    	

	    	data.setIsParameter(cdr.read_type_7());
	    	

	    	data.setMin(cdr.read_type_6());
	    	

	    	data.setMax(cdr.read_type_6());
	    	

	    	data.setLoadStatus(us.ihmc.robotDataLogger.LoadStatus.values[cdr.read_type_c()]);
	    	
   }
   
	@Override
	public final void serialize(us.ihmc.robotDataLogger.YoVariableDefinition data, us.ihmc.idl.InterchangeSerializer ser)
	{
			    ser.write_type_d("name", data.getName());
			    
			    ser.write_type_d("description", data.getDescription());
			    
			    ser.write_type_c("type", data.getType());
			    
			    ser.write_type_3("registry", data.getRegistry());
			    
			    ser.write_type_3("enumType", data.getEnumType());
			    
			    ser.write_type_7("allowNullValues", data.getAllowNullValues());
			    
			    ser.write_type_7("isParameter", data.getIsParameter());
			    
			    ser.write_type_6("min", data.getMin());
			    
			    ser.write_type_6("max", data.getMax());
			    
			    ser.write_type_c("loadStatus", data.getLoadStatus());
			    
	}
	
	@Override
	public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, us.ihmc.robotDataLogger.YoVariableDefinition data)
	{
	    			ser.read_type_d("name", data.getName());	
	    	    
	    			ser.read_type_d("description", data.getDescription());	
	    	    
	    			data.setType((us.ihmc.robotDataLogger.YoType)ser.read_type_c("type", us.ihmc.robotDataLogger.YoType.class));
	    	
	    	    
	    			data.setRegistry(ser.read_type_3("registry"));	
	    	    
	    			data.setEnumType(ser.read_type_3("enumType"));	
	    	    
	    			data.setAllowNullValues(ser.read_type_7("allowNullValues"));	
	    	    
	    			data.setIsParameter(ser.read_type_7("isParameter"));	
	    	    
	    			data.setMin(ser.read_type_6("min"));	
	    	    
	    			data.setMax(ser.read_type_6("max"));	
	    	    
	    			data.setLoadStatus((us.ihmc.robotDataLogger.LoadStatus)ser.read_type_c("loadStatus", us.ihmc.robotDataLogger.LoadStatus.class));
	    	
	    	    
	}

   public static void staticCopy(us.ihmc.robotDataLogger.YoVariableDefinition src, us.ihmc.robotDataLogger.YoVariableDefinition dest)
   {
      dest.set(src);
   }
   
   
   @Override
   public us.ihmc.robotDataLogger.YoVariableDefinition createData()
   {
      return new us.ihmc.robotDataLogger.YoVariableDefinition();
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
   
   public void serialize(us.ihmc.robotDataLogger.YoVariableDefinition data, us.ihmc.idl.CDR cdr)
	{
		write(data, cdr);
	}

   public void deserialize(us.ihmc.robotDataLogger.YoVariableDefinition data, us.ihmc.idl.CDR cdr)
   {
        read(data, cdr);
   }
   
   public void copy(us.ihmc.robotDataLogger.YoVariableDefinition src, us.ihmc.robotDataLogger.YoVariableDefinition dest)
   {
      staticCopy(src, dest);
   }	

   
   @Override
   public YoVariableDefinitionPubSubType newInstance()
   {
   	  return new YoVariableDefinitionPubSubType();
   }
}