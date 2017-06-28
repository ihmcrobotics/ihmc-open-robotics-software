package us.ihmc.robotDataLogger;

import java.io.IOException;

import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.common.SerializedPayload;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.idl.CDR;
import us.ihmc.idl.IDLSequence;

/**
* 
* Topic data type of the struct "Announcement" defined in "Announcement.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Announcement.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Announcement.idl instead.
*
*/
public class AnnouncementPubSubType implements TopicDataType<us.ihmc.robotDataLogger.Announcement>
{
	public static final String name = "us::ihmc::robotDataLogger::Announcement";
	
	
	
    public AnnouncementPubSubType()
    {
        
    }

	private final CDR serializeCDR = new CDR();
	private final CDR deserializeCDR = new CDR();

    
    @Override
   public void serialize(us.ihmc.robotDataLogger.Announcement data, SerializedPayload serializedPayload) throws IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }
   @Override
   public void deserialize(SerializedPayload serializedPayload, us.ihmc.robotDataLogger.Announcement data) throws IOException
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
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += ((4) * 1) + CDR.alignment(current_alignment, 1);

	    current_alignment += 2 + CDR.alignment(current_alignment, 2);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < 127; ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.CameraAnnouncementPubSubType.getMaxCdrSerializedSize(current_alignment);}

	    current_alignment += us.ihmc.robotDataLogger.ModelFileDescriptionPubSubType.getMaxCdrSerializedSize(current_alignment);
	    current_alignment += 1 + CDR.alignment(current_alignment, 1);

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.Announcement data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.Announcement data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getIdentifier().length() + 1;

	    current_alignment += ((4) * 1) + CDR.alignment(current_alignment, 1);
	    current_alignment += 2 + CDR.alignment(current_alignment, 2);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getName().length() + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getHostName().length() + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < data.getCameras().size(); ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.CameraAnnouncementPubSubType.getCdrSerializedSize(data.getCameras().get(a), current_alignment);}

	    current_alignment += us.ihmc.robotDataLogger.ModelFileDescriptionPubSubType.getCdrSerializedSize(data.getModelFileDescription(), current_alignment);
	    current_alignment += 1 + CDR.alignment(current_alignment, 1);

	
	    return current_alignment - initial_alignment;
	}
	
   public static void write(us.ihmc.robotDataLogger.Announcement data, CDR cdr)
   {

	    if(data.getIdentifier().length() <= 255)
	    cdr.write_type_d(data.getIdentifier());else
	        throw new RuntimeException("identifier field exceeds the maximum length");

	    for(int a = 0; a < data.getDataIP().length; ++a)
	    {
	        	cdr.write_type_9(data.getDataIP()[a]);	
	    }


	    cdr.write_type_3(data.getDataPort());

	    if(data.getName().length() <= 255)
	    cdr.write_type_d(data.getName());else
	        throw new RuntimeException("name field exceeds the maximum length");

	    if(data.getHostName().length() <= 255)
	    cdr.write_type_d(data.getHostName());else
	        throw new RuntimeException("hostName field exceeds the maximum length");

	    if(data.getCameras().size() <= 127)
	    cdr.write_type_e(data.getCameras());else
	        throw new RuntimeException("cameras field exceeds the maximum length");

	    us.ihmc.robotDataLogger.ModelFileDescriptionPubSubType.write(data.getModelFileDescription(), cdr);

	    cdr.write_type_7(data.getLog());
   }

   public static void read(us.ihmc.robotDataLogger.Announcement data, CDR cdr)
   {

	    	cdr.read_type_d(data.getIdentifier());	

	    	for(int a = 0; a < data.getDataIP().length; ++a)
	    	{
	    	    	data.getDataIP()[a] = cdr.read_type_9();
	    	    	
	    	}
	    	

	    	data.setDataPort(cdr.read_type_3());
	    	

	    	cdr.read_type_d(data.getName());	

	    	cdr.read_type_d(data.getHostName());	

	    	cdr.read_type_e(data.getCameras());	

	    	us.ihmc.robotDataLogger.ModelFileDescriptionPubSubType.read(data.getModelFileDescription(), cdr);	

	    	data.setLog(cdr.read_type_7());
	    	
   }
   
	@Override
	public final void serialize(us.ihmc.robotDataLogger.Announcement data, InterchangeSerializer ser)
	{
			    ser.write_type_d("identifier", data.getIdentifier());
			    
				    	ser.write_type_f("dataIP", data.getDataIP());	    
			    ser.write_type_3("dataPort", data.getDataPort());
			    
			    ser.write_type_d("name", data.getName());
			    
			    ser.write_type_d("hostName", data.getHostName());
			    
			    ser.write_type_e("cameras", data.getCameras());
			    
			    ser.write_type_a("modelFileDescription", new us.ihmc.robotDataLogger.ModelFileDescriptionPubSubType(), data.getModelFileDescription());

			    
			    ser.write_type_7("log", data.getLog());
			    
	}
	
	@Override
	public final void deserialize(InterchangeSerializer ser, us.ihmc.robotDataLogger.Announcement data)
	{
	    			ser.read_type_d("identifier", data.getIdentifier());	
	    	    
	    		    	ser.read_type_f("dataIP", data.getDataIP());			
	    	
	    	    
	    			data.setDataPort(ser.read_type_3("dataPort"));	
	    	    
	    			ser.read_type_d("name", data.getName());	
	    	    
	    			ser.read_type_d("hostName", data.getHostName());	
	    	    
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
      return CDR.getTypeSize(getMaxCdrSerializedSize());
   }

   @Override
   public String getName()
   {
      return name;
   }
   
   public void serialize(us.ihmc.robotDataLogger.Announcement data, CDR cdr)
	{
		write(data, cdr);
	}

   public void deserialize(us.ihmc.robotDataLogger.Announcement data, CDR cdr)
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