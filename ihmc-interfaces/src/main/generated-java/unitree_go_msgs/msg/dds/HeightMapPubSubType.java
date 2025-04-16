package unitree_go_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HeightMap" defined in "HeightMap_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HeightMap_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HeightMap_.idl instead.
*
*/
public class HeightMapPubSubType implements us.ihmc.pubsub.TopicDataType<unitree_go_msgs.msg.dds.HeightMap>
{
   public static final java.lang.String name = "unitree_go_msgs::msg::dds_::HeightMap_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "9abcbbb26f636e577c562dfec1b1bd1ba4c93ebb33d75c675d596b80254a81a6";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(unitree_go_msgs.msg.dds.HeightMap data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, unitree_go_msgs.msg.dds.HeightMap data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.HeightMap data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.HeightMap data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getFrameId().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getData().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(unitree_go_msgs.msg.dds.HeightMap data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_6(data.getStamp());

      if(data.getFrameId().length() <= 255)
      cdr.write_type_d(data.getFrameId());else
          throw new RuntimeException("frame_id field exceeds the maximum length: %d > %d".formatted(data.getFrameId().length(), 255));

      cdr.write_type_5(data.getResolution());

      cdr.write_type_4(data.getWidth());

      cdr.write_type_4(data.getHeight());

      for(int i0 = 0; i0 < data.getOrigin().length; ++i0)
      {
        	cdr.write_type_5(data.getOrigin()[i0]);	
      }

      if(data.getData().size() <= 100)
      cdr.write_type_e(data.getData());else
          throw new RuntimeException("data field exceeds the maximum length: %d > %d".formatted(data.getData().size(), 100));

   }

   public static void read(unitree_go_msgs.msg.dds.HeightMap data, us.ihmc.idl.CDR cdr)
   {
      data.setStamp(cdr.read_type_6());
      	
      cdr.read_type_d(data.getFrameId());	
      data.setResolution(cdr.read_type_5());
      	
      data.setWidth(cdr.read_type_4());
      	
      data.setHeight(cdr.read_type_4());
      	
      for(int i0 = 0; i0 < data.getOrigin().length; ++i0)
      {
        	data.getOrigin()[i0] = cdr.read_type_5();
        	
      }
      	
      cdr.read_type_e(data.getData());	

   }

   @Override
   public final void serialize(unitree_go_msgs.msg.dds.HeightMap data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_6("stamp", data.getStamp());
      ser.write_type_d("frame_id", data.getFrameId());
      ser.write_type_5("resolution", data.getResolution());
      ser.write_type_4("width", data.getWidth());
      ser.write_type_4("height", data.getHeight());
      ser.write_type_f("origin", data.getOrigin());
      ser.write_type_e("data", data.getData());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, unitree_go_msgs.msg.dds.HeightMap data)
   {
      data.setStamp(ser.read_type_6("stamp"));
      ser.read_type_d("frame_id", data.getFrameId());
      data.setResolution(ser.read_type_5("resolution"));
      data.setWidth(ser.read_type_4("width"));
      data.setHeight(ser.read_type_4("height"));
      ser.read_type_f("origin", data.getOrigin());
      ser.read_type_e("data", data.getData());
   }

   public static void staticCopy(unitree_go_msgs.msg.dds.HeightMap src, unitree_go_msgs.msg.dds.HeightMap dest)
   {
      dest.set(src);
   }

   @Override
   public unitree_go_msgs.msg.dds.HeightMap createData()
   {
      return new unitree_go_msgs.msg.dds.HeightMap();
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
   
   public void serialize(unitree_go_msgs.msg.dds.HeightMap data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(unitree_go_msgs.msg.dds.HeightMap data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(unitree_go_msgs.msg.dds.HeightMap src, unitree_go_msgs.msg.dds.HeightMap dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HeightMapPubSubType newInstance()
   {
      return new HeightMapPubSubType();
   }
}
