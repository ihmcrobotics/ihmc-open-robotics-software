package us.ihmc.robotDataLogger;

/**
* 
* Topic data type of the struct "GraphicObjectMessage" defined in "Handshake.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
public class GraphicObjectMessagePubSubType implements us.ihmc.pubsub.TopicDataType<us.ihmc.robotDataLogger.GraphicObjectMessage>
{
	public static final java.lang.String name = "us::ihmc::robotDataLogger::GraphicObjectMessage";
	
	
	
    public GraphicObjectMessagePubSubType()
    {
        
    }

	private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
	private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

    
    @Override
   public void serialize(us.ihmc.robotDataLogger.GraphicObjectMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }
   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, us.ihmc.robotDataLogger.GraphicObjectMessage data) throws java.io.IOException
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

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
	    current_alignment += (1024 * 2) + us.ihmc.idl.CDR.alignment(current_alignment, 2);


	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
	    current_alignment += (128 * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);


	    current_alignment += us.ihmc.robotDataLogger.AppearanceDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);
	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.GraphicObjectMessage data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.GraphicObjectMessage data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getName().length() + 1;

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
	    current_alignment += (data.getYoVariableIndex().size() * 2) + us.ihmc.idl.CDR.alignment(current_alignment, 2);


	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
	    current_alignment += (data.getConstants().size() * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);


	    current_alignment += us.ihmc.robotDataLogger.AppearanceDefinitionMessagePubSubType.getCdrSerializedSize(data.getAppearance(), current_alignment);
	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getListName().length() + 1;

	
	    return current_alignment - initial_alignment;
	}
	
   public static void write(us.ihmc.robotDataLogger.GraphicObjectMessage data, us.ihmc.idl.CDR cdr)
   {

	    cdr.write_type_3(data.getType());

	    if(data.getName().length() <= 255)
	    cdr.write_type_d(data.getName());else
	        throw new RuntimeException("name field exceeds the maximum length");

	    if(data.getYoVariableIndex().size() <= 1024)
	    cdr.write_type_e(data.getYoVariableIndex());else
	        throw new RuntimeException("yoVariableIndex field exceeds the maximum length");

	    if(data.getConstants().size() <= 128)
	    cdr.write_type_e(data.getConstants());else
	        throw new RuntimeException("constants field exceeds the maximum length");

	    us.ihmc.robotDataLogger.AppearanceDefinitionMessagePubSubType.write(data.getAppearance(), cdr);

	    if(data.getListName().length() <= 255)
	    cdr.write_type_d(data.getListName());else
	        throw new RuntimeException("listName field exceeds the maximum length");
   }

   public static void read(us.ihmc.robotDataLogger.GraphicObjectMessage data, us.ihmc.idl.CDR cdr)
   {

	    	data.setType(cdr.read_type_3());
	    	

	    	cdr.read_type_d(data.getName());	

	    	cdr.read_type_e(data.getYoVariableIndex());	

	    	cdr.read_type_e(data.getConstants());	

	    	us.ihmc.robotDataLogger.AppearanceDefinitionMessagePubSubType.read(data.getAppearance(), cdr);	

	    	cdr.read_type_d(data.getListName());	
   }
   
	@Override
	public final void serialize(us.ihmc.robotDataLogger.GraphicObjectMessage data, us.ihmc.idl.InterchangeSerializer ser)
	{
			    ser.write_type_3("type", data.getType());
			    
			    ser.write_type_d("name", data.getName());
			    
			    ser.write_type_e("yoVariableIndex", data.getYoVariableIndex());
			    
			    ser.write_type_e("constants", data.getConstants());
			    
			    ser.write_type_a("appearance", new us.ihmc.robotDataLogger.AppearanceDefinitionMessagePubSubType(), data.getAppearance());

			    
			    ser.write_type_d("listName", data.getListName());
			    
	}
	
	@Override
	public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, us.ihmc.robotDataLogger.GraphicObjectMessage data)
	{
	    			data.setType(ser.read_type_3("type"));	
	    	    
	    			ser.read_type_d("name", data.getName());	
	    	    
	    			ser.read_type_e("yoVariableIndex", data.getYoVariableIndex());	
	    	    
	    			ser.read_type_e("constants", data.getConstants());	
	    	    
	    			ser.read_type_a("appearance", new us.ihmc.robotDataLogger.AppearanceDefinitionMessagePubSubType(), data.getAppearance());
	    	
	    	    
	    			ser.read_type_d("listName", data.getListName());	
	    	    
	}

   public static void staticCopy(us.ihmc.robotDataLogger.GraphicObjectMessage src, us.ihmc.robotDataLogger.GraphicObjectMessage dest)
   {
      dest.set(src);
   }
   
   
   @Override
   public us.ihmc.robotDataLogger.GraphicObjectMessage createData()
   {
      return new us.ihmc.robotDataLogger.GraphicObjectMessage();
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
   
   public void serialize(us.ihmc.robotDataLogger.GraphicObjectMessage data, us.ihmc.idl.CDR cdr)
	{
		write(data, cdr);
	}

   public void deserialize(us.ihmc.robotDataLogger.GraphicObjectMessage data, us.ihmc.idl.CDR cdr)
   {
        read(data, cdr);
   }
   
   public void copy(us.ihmc.robotDataLogger.GraphicObjectMessage src, us.ihmc.robotDataLogger.GraphicObjectMessage dest)
   {
      staticCopy(src, dest);
   }	

   
   @Override
   public GraphicObjectMessagePubSubType newInstance()
   {
   	  return new GraphicObjectMessagePubSubType();
   }
}