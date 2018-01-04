package us.ihmc.robotDataLogger;

/**
* 
* Topic data type of the struct "Camera" defined in "LogProperties.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from LogProperties.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LogProperties.idl instead.
*
*/
public class CameraPubSubType implements us.ihmc.pubsub.TopicDataType<us.ihmc.robotDataLogger.Camera>
{
	public static final java.lang.String name = "us::ihmc::robotDataLogger::Camera";
	
	
	
    public CameraPubSubType()
    {
        
    }

	private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
	private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

    
    @Override
   public void serialize(us.ihmc.robotDataLogger.Camera data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }
   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, us.ihmc.robotDataLogger.Camera data) throws java.io.IOException
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

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.Camera data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.Camera data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getName().length() + 1;

	    current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getVideoFile().length() + 1;

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getTimestampFile().length() + 1;

	
	    return current_alignment - initial_alignment;
	}
	
   public static void write(us.ihmc.robotDataLogger.Camera data, us.ihmc.idl.CDR cdr)
   {

	    if(data.getName().length() <= 255)
	    cdr.write_type_d(data.getName());else
	        throw new RuntimeException("name field exceeds the maximum length");

	    cdr.write_type_7(data.getInterlaced());

	    if(data.getVideoFile().length() <= 255)
	    cdr.write_type_d(data.getVideoFile());else
	        throw new RuntimeException("videoFile field exceeds the maximum length");

	    if(data.getTimestampFile().length() <= 255)
	    cdr.write_type_d(data.getTimestampFile());else
	        throw new RuntimeException("timestampFile field exceeds the maximum length");
   }

   public static void read(us.ihmc.robotDataLogger.Camera data, us.ihmc.idl.CDR cdr)
   {

	    	cdr.read_type_d(data.getName());	

	    	data.setInterlaced(cdr.read_type_7());
	    	

	    	cdr.read_type_d(data.getVideoFile());	

	    	cdr.read_type_d(data.getTimestampFile());	
   }
   
	@Override
	public final void serialize(us.ihmc.robotDataLogger.Camera data, us.ihmc.idl.InterchangeSerializer ser)
	{
			    ser.write_type_d("name", data.getName());
			    
			    ser.write_type_7("interlaced", data.getInterlaced());
			    
			    ser.write_type_d("videoFile", data.getVideoFile());
			    
			    ser.write_type_d("timestampFile", data.getTimestampFile());
			    
	}
	
	@Override
	public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, us.ihmc.robotDataLogger.Camera data)
	{
	    			ser.read_type_d("name", data.getName());	
	    	    
	    			data.setInterlaced(ser.read_type_7("interlaced"));	
	    	    
	    			ser.read_type_d("videoFile", data.getVideoFile());	
	    	    
	    			ser.read_type_d("timestampFile", data.getTimestampFile());	
	    	    
	}

   public static void staticCopy(us.ihmc.robotDataLogger.Camera src, us.ihmc.robotDataLogger.Camera dest)
   {
      dest.set(src);
   }
   
   
   @Override
   public us.ihmc.robotDataLogger.Camera createData()
   {
      return new us.ihmc.robotDataLogger.Camera();
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
   
   public void serialize(us.ihmc.robotDataLogger.Camera data, us.ihmc.idl.CDR cdr)
	{
		write(data, cdr);
	}

   public void deserialize(us.ihmc.robotDataLogger.Camera data, us.ihmc.idl.CDR cdr)
   {
        read(data, cdr);
   }
   
   public void copy(us.ihmc.robotDataLogger.Camera src, us.ihmc.robotDataLogger.Camera dest)
   {
      staticCopy(src, dest);
   }	

   
   @Override
   public CameraPubSubType newInstance()
   {
   	  return new CameraPubSubType();
   }
}