package us.ihmc.robotDataLogger;

/**
* 
* Topic data type of the struct "Announcement" defined in "Announcement.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Announcement.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Announcement.idl instead.
*
*/
public class AnnouncementPubSubType implements us.ihmc.pubsub.TopicDataType<us.ihmc.robotDataLogger.Announcement>
{
	public static final java.lang.String name = "us::ihmc::robotDataLogger::Announcement";
	
	
	
    public AnnouncementPubSubType()
    {
        
    }

	private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
	private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

    
    @Override
   public void serialize(us.ihmc.robotDataLogger.Announcement data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }
   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, us.ihmc.robotDataLogger.Announcement data) throws java.io.IOException
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

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < 127; ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.CameraAnnouncementPubSubType.getMaxCdrSerializedSize(current_alignment);}

	    current_alignment += us.ihmc.robotDataLogger.ModelFileDescriptionPubSubType.getMaxCdrSerializedSize(current_alignment);
	    current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.Announcement data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.Announcement data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getIdentifier().length() + 1;

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getName().length() + 1;

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getHostName().length() + 1;

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getReconnectKey().length() + 1;

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < data.getCameras().size(); ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.CameraAnnouncementPubSubType.getCdrSerializedSize(data.getCameras().get(a), current_alignment);}

	    current_alignment += us.ihmc.robotDataLogger.ModelFileDescriptionPubSubType.getCdrSerializedSize(data.getModelFileDescription(), current_alignment);
	    current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

	
	    return current_alignment - initial_alignment;
	}
	
   public static void write(us.ihmc.robotDataLogger.Announcement data, us.ihmc.idl.CDR cdr)
   {

	    if(data.getIdentifier().length() <= 255)
	    cdr.write_type_d(data.getIdentifier());else
	        throw new RuntimeException("identifier field exceeds the maximum length");

	    if(data.getName().length() <= 255)
	    cdr.write_type_d(data.getName());else
	        throw new RuntimeException("name field exceeds the maximum length");

	    if(data.getHostName().length() <= 255)
	    cdr.write_type_d(data.getHostName());else
	        throw new RuntimeException("hostName field exceeds the maximum length");

	    if(data.getReconnectKey().length() <= 255)
	    cdr.write_type_d(data.getReconnectKey());else
	        throw new RuntimeException("reconnectKey field exceeds the maximum length");

	    if(data.getCameras().size() <= 127)
	    cdr.write_type_e(data.getCameras());else
	        throw new RuntimeException("cameras field exceeds the maximum length");

	    us.ihmc.robotDataLogger.ModelFileDescriptionPubSubType.write(data.getModelFileDescription(), cdr);

	    cdr.write_type_7(data.getLog());
   }

   public static void read(us.ihmc.robotDataLogger.Announcement data, us.ihmc.idl.CDR cdr)
   {

	    	cdr.read_type_d(data.getIdentifier());	

	    	cdr.read_type_d(data.getName());	

	    	cdr.read_type_d(data.getHostName());	

	    	cdr.read_type_d(data.getReconnectKey());	

	    	cdr.read_type_e(data.getCameras());	

	    	us.ihmc.robotDataLogger.ModelFileDescriptionPubSubType.read(data.getModelFileDescription(), cdr);	

	    	data.setLog(cdr.read_type_7());
	    	
   }
   
	@Override
	public final void serialize(us.ihmc.robotDataLogger.Announcement data, us.ihmc.idl.InterchangeSerializer ser)
	{
			    ser.write_type_d("identifier", data.getIdentifier());
			    
			    ser.write_type_d("name", data.getName());
			    
			    ser.write_type_d("hostName", data.getHostName());
			    
			    ser.write_type_d("reconnectKey", data.getReconnectKey());
			    
			    ser.write_type_e("cameras", data.getCameras());
			    
			    ser.write_type_a("modelFileDescription", new us.ihmc.robotDataLogger.ModelFileDescriptionPubSubType(), data.getModelFileDescription());

			    
			    ser.write_type_7("log", data.getLog());
			    
	}
	
	@Override
	public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, us.ihmc.robotDataLogger.Announcement data)
	{
	    			ser.read_type_d("identifier", data.getIdentifier());	
	    	    
	    			ser.read_type_d("name", data.getName());	
	    	    
	    			ser.read_type_d("hostName", data.getHostName());	
	    	    
	    			ser.read_type_d("reconnectKey", data.getReconnectKey());	
	    	    
	    			ser.read_type_e("cameras", data.getCameras());	
	    	    
	    			ser.read_type_a("modelFileDescription", new us.ihmc.robotDataLogger.ModelFileDescriptionPubSubType(), data.getModelFileDescription());
	    	
	    	    
	    			data.setLog(ser.read_type_7("log"));	
	    	    
	}

   public static void staticCopy(us.ihmc.robotDataLogger.Announcement src, us.ihmc.robotDataLogger.Announcement dest)
   {
      dest.set(src);
   }
   
   
   @Override
   public us.ihmc.robotDataLogger.Announcement createData()
   {
      return new us.ihmc.robotDataLogger.Announcement();
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
   
   public void serialize(us.ihmc.robotDataLogger.Announcement data, us.ihmc.idl.CDR cdr)
	{
		write(data, cdr);
	}

   public void deserialize(us.ihmc.robotDataLogger.Announcement data, us.ihmc.idl.CDR cdr)
   {
        read(data, cdr);
   }
   
   public void copy(us.ihmc.robotDataLogger.Announcement src, us.ihmc.robotDataLogger.Announcement dest)
   {
      staticCopy(src, dest);
   }	

   
   @Override
   public AnnouncementPubSubType newInstance()
   {
   	  return new AnnouncementPubSubType();
   }
}