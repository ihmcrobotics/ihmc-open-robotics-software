package us.ihmc.robotDataLogger;

/**
* 
* Topic data type of the struct "YoRegistryDefinition" defined in "Handshake.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
public class YoRegistryDefinitionPubSubType implements us.ihmc.pubsub.TopicDataType<us.ihmc.robotDataLogger.YoRegistryDefinition>
{
	public static final java.lang.String name = "us::ihmc::robotDataLogger::YoRegistryDefinition";
	
	
	
    public YoRegistryDefinitionPubSubType()
    {
        
    }

	private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
	private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

    
    @Override
   public void serialize(us.ihmc.robotDataLogger.YoRegistryDefinition data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }
   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, us.ihmc.robotDataLogger.YoRegistryDefinition data) throws java.io.IOException
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
	            
	    current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.YoRegistryDefinition data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.YoRegistryDefinition data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getName().length() + 1;

	
	    return current_alignment - initial_alignment;
	}
	
   public static void write(us.ihmc.robotDataLogger.YoRegistryDefinition data, us.ihmc.idl.CDR cdr)
   {

	    cdr.write_type_3(data.getParent());

	    if(data.getName().length() <= 255)
	    cdr.write_type_d(data.getName());else
	        throw new RuntimeException("name field exceeds the maximum length");
   }

   public static void read(us.ihmc.robotDataLogger.YoRegistryDefinition data, us.ihmc.idl.CDR cdr)
   {

	    	data.setParent(cdr.read_type_3());
	    	

	    	cdr.read_type_d(data.getName());	
   }
   
	@Override
	public final void serialize(us.ihmc.robotDataLogger.YoRegistryDefinition data, us.ihmc.idl.InterchangeSerializer ser)
	{
			    ser.write_type_3("parent", data.getParent());
			    
			    ser.write_type_d("name", data.getName());
			    
	}
	
	@Override
	public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, us.ihmc.robotDataLogger.YoRegistryDefinition data)
	{
	    			data.setParent(ser.read_type_3("parent"));	
	    	    
	    			ser.read_type_d("name", data.getName());	
	    	    
	}

   public static void staticCopy(us.ihmc.robotDataLogger.YoRegistryDefinition src, us.ihmc.robotDataLogger.YoRegistryDefinition dest)
   {
      dest.set(src);
   }
   
   
   @Override
   public us.ihmc.robotDataLogger.YoRegistryDefinition createData()
   {
      return new us.ihmc.robotDataLogger.YoRegistryDefinition();
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
   
   public void serialize(us.ihmc.robotDataLogger.YoRegistryDefinition data, us.ihmc.idl.CDR cdr)
	{
		write(data, cdr);
	}

   public void deserialize(us.ihmc.robotDataLogger.YoRegistryDefinition data, us.ihmc.idl.CDR cdr)
   {
        read(data, cdr);
   }
   
   public void copy(us.ihmc.robotDataLogger.YoRegistryDefinition src, us.ihmc.robotDataLogger.YoRegistryDefinition dest)
   {
      staticCopy(src, dest);
   }	

   
   @Override
   public YoRegistryDefinitionPubSubType newInstance()
   {
   	  return new YoRegistryDefinitionPubSubType();
   }
}