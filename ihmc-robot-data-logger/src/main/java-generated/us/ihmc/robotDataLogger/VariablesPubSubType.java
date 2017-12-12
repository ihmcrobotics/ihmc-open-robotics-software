package us.ihmc.robotDataLogger;

/**
* 
* Topic data type of the struct "Variables" defined in "LogProperties.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from LogProperties.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LogProperties.idl instead.
*
*/
public class VariablesPubSubType implements us.ihmc.pubsub.TopicDataType<us.ihmc.robotDataLogger.Variables>
{
	public static final java.lang.String name = "us::ihmc::robotDataLogger::Variables";
	
	
	
    public VariablesPubSubType()
    {
        
    }

	private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
	private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

    
    @Override
   public void serialize(us.ihmc.robotDataLogger.Variables data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }
   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, us.ihmc.robotDataLogger.Variables data) throws java.io.IOException
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
	            
	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

	    current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.Variables data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.Variables data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getHandshake().length() + 1;

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getData().length() + 1;

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getSummary().length() + 1;

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getIndex().length() + 1;

	    current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

	    current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

	
	    return current_alignment - initial_alignment;
	}
	
   public static void write(us.ihmc.robotDataLogger.Variables data, us.ihmc.idl.CDR cdr)
   {

	    cdr.write_type_c(data.getHandshakeFileType().ordinal());


	    if(data.getHandshake().length() <= 255)
	    cdr.write_type_d(data.getHandshake());else
	        throw new RuntimeException("handshake field exceeds the maximum length");

	    if(data.getData().length() <= 255)
	    cdr.write_type_d(data.getData());else
	        throw new RuntimeException("data field exceeds the maximum length");

	    if(data.getSummary().length() <= 255)
	    cdr.write_type_d(data.getSummary());else
	        throw new RuntimeException("summary field exceeds the maximum length");

	    if(data.getIndex().length() <= 255)
	    cdr.write_type_d(data.getIndex());else
	        throw new RuntimeException("index field exceeds the maximum length");

	    cdr.write_type_7(data.getTimestamped());

	    cdr.write_type_7(data.getCompressed());
   }

   public static void read(us.ihmc.robotDataLogger.Variables data, us.ihmc.idl.CDR cdr)
   {

	    	data.setHandshakeFileType(us.ihmc.robotDataLogger.HandshakeFileType.values[cdr.read_type_c()]);
	    	

	    	cdr.read_type_d(data.getHandshake());	

	    	cdr.read_type_d(data.getData());	

	    	cdr.read_type_d(data.getSummary());	

	    	cdr.read_type_d(data.getIndex());	

	    	data.setTimestamped(cdr.read_type_7());
	    	

	    	data.setCompressed(cdr.read_type_7());
	    	
   }
   
	@Override
	public final void serialize(us.ihmc.robotDataLogger.Variables data, us.ihmc.idl.InterchangeSerializer ser)
	{
			    ser.write_type_c("handshakeFileType", data.getHandshakeFileType());
			    
			    ser.write_type_d("handshake", data.getHandshake());
			    
			    ser.write_type_d("data", data.getData());
			    
			    ser.write_type_d("summary", data.getSummary());
			    
			    ser.write_type_d("index", data.getIndex());
			    
			    ser.write_type_7("timestamped", data.getTimestamped());
			    
			    ser.write_type_7("compressed", data.getCompressed());
			    
	}
	
	@Override
	public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, us.ihmc.robotDataLogger.Variables data)
	{
	    			data.setHandshakeFileType((us.ihmc.robotDataLogger.HandshakeFileType)ser.read_type_c("handshakeFileType", us.ihmc.robotDataLogger.HandshakeFileType.class));
	    	
	    	    
	    			ser.read_type_d("handshake", data.getHandshake());	
	    	    
	    			ser.read_type_d("data", data.getData());	
	    	    
	    			ser.read_type_d("summary", data.getSummary());	
	    	    
	    			ser.read_type_d("index", data.getIndex());	
	    	    
	    			data.setTimestamped(ser.read_type_7("timestamped"));	
	    	    
	    			data.setCompressed(ser.read_type_7("compressed"));	
	    	    
	}

   public static void staticCopy(us.ihmc.robotDataLogger.Variables src, us.ihmc.robotDataLogger.Variables dest)
   {
      dest.set(src);
   }
   
   
   @Override
   public us.ihmc.robotDataLogger.Variables createData()
   {
      return new us.ihmc.robotDataLogger.Variables();
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
   
   public void serialize(us.ihmc.robotDataLogger.Variables data, us.ihmc.idl.CDR cdr)
	{
		write(data, cdr);
	}

   public void deserialize(us.ihmc.robotDataLogger.Variables data, us.ihmc.idl.CDR cdr)
   {
        read(data, cdr);
   }
   
   public void copy(us.ihmc.robotDataLogger.Variables src, us.ihmc.robotDataLogger.Variables dest)
   {
      staticCopy(src, dest);
   }	

   
   @Override
   public VariablesPubSubType newInstance()
   {
   	  return new VariablesPubSubType();
   }
}