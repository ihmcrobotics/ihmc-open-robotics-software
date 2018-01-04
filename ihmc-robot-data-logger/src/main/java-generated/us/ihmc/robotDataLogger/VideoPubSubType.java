package us.ihmc.robotDataLogger;

/**
* 
* Topic data type of the struct "Video" defined in "LogProperties.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from LogProperties.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LogProperties.idl instead.
*
*/
public class VideoPubSubType implements us.ihmc.pubsub.TopicDataType<us.ihmc.robotDataLogger.Video>
{
	public static final java.lang.String name = "us::ihmc::robotDataLogger::Video";
	
	
	
    public VideoPubSubType()
    {
        
    }

	private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
	private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

    
    @Override
   public void serialize(us.ihmc.robotDataLogger.Video data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }
   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, us.ihmc.robotDataLogger.Video data) throws java.io.IOException
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
	            
	    current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.Video data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.Video data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

	
	    return current_alignment - initial_alignment;
	}
	
   public static void write(us.ihmc.robotDataLogger.Video data, us.ihmc.idl.CDR cdr)
   {

	    cdr.write_type_7(data.getHasTimebase());
   }

   public static void read(us.ihmc.robotDataLogger.Video data, us.ihmc.idl.CDR cdr)
   {

	    	data.setHasTimebase(cdr.read_type_7());
	    	
   }
   
	@Override
	public final void serialize(us.ihmc.robotDataLogger.Video data, us.ihmc.idl.InterchangeSerializer ser)
	{
			    ser.write_type_7("hasTimebase", data.getHasTimebase());
			    
	}
	
	@Override
	public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, us.ihmc.robotDataLogger.Video data)
	{
	    			data.setHasTimebase(ser.read_type_7("hasTimebase"));	
	    	    
	}

   public static void staticCopy(us.ihmc.robotDataLogger.Video src, us.ihmc.robotDataLogger.Video dest)
   {
      dest.set(src);
   }
   
   
   @Override
   public us.ihmc.robotDataLogger.Video createData()
   {
      return new us.ihmc.robotDataLogger.Video();
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
   
   public void serialize(us.ihmc.robotDataLogger.Video data, us.ihmc.idl.CDR cdr)
	{
		write(data, cdr);
	}

   public void deserialize(us.ihmc.robotDataLogger.Video data, us.ihmc.idl.CDR cdr)
   {
        read(data, cdr);
   }
   
   public void copy(us.ihmc.robotDataLogger.Video src, us.ihmc.robotDataLogger.Video dest)
   {
      staticCopy(src, dest);
   }	

   
   @Override
   public VideoPubSubType newInstance()
   {
   	  return new VideoPubSubType();
   }
}