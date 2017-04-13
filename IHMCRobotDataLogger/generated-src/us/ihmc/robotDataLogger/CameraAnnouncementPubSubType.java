package us.ihmc.robotDataLogger;

import java.io.IOException;

import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.common.SerializedPayload;
import us.ihmc.idl.CDR;

/**
* 
* Topic data type of the struct "CameraAnnouncement" defined in "Announcement.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Announcement.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Announcement.idl instead.
*
*/
public class CameraAnnouncementPubSubType implements TopicDataType<CameraAnnouncement>
{
	public static final String name = "us::ihmc::robotDataLogger::CameraAnnouncement";
	private final CDR serializeCDR = new CDR();
	private final CDR deserializeCDR = new CDR();
	
	
	
    public CameraAnnouncementPubSubType()
    {
        
    }
    
       @Override
   public void serialize(CameraAnnouncement data, SerializedPayload serializedPayload) throws IOException
   {
      serializeCDR.serialize(serializedPayload);
      data.serialize(serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(SerializedPayload serializedPayload, CameraAnnouncement data) throws IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      data.deserialize(deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public int getTypeSize()
   {
      return CDR.getTypeSize(CameraAnnouncement.getMaxCdrSerializedSize());
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public CameraAnnouncement createData()
   {
      return new CameraAnnouncement();
   }
   
   @Override
   public CameraAnnouncementPubSubType newInstance()
   {
   	  return new CameraAnnouncementPubSubType();
   }
}