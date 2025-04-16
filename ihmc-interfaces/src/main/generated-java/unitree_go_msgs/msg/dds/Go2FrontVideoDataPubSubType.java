package unitree_go_msgs.msg.dds;

/**
* 
* Topic data type of the struct "Go2FrontVideoData" defined in "Go2FrontVideoData_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Go2FrontVideoData_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Go2FrontVideoData_.idl instead.
*
*/
public class Go2FrontVideoDataPubSubType implements us.ihmc.pubsub.TopicDataType<unitree_go_msgs.msg.dds.Go2FrontVideoData>
{
   public static final java.lang.String name = "unitree_go_msgs::msg::dds_::Go2FrontVideoData_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "92b767e8c7326215f310822694988f4a1f7dd0ad6ef7c44ef5d1960dcc183abb";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(unitree_go_msgs.msg.dds.Go2FrontVideoData data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, unitree_go_msgs.msg.dds.Go2FrontVideoData data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.Go2FrontVideoData data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.Go2FrontVideoData data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getVideo720p().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getVideo360p().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getVideo180p().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(unitree_go_msgs.msg.dds.Go2FrontVideoData data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_12(data.getTimeFrame());

      if(data.getVideo720p().size() <= 100)
      cdr.write_type_e(data.getVideo720p());else
          throw new RuntimeException("video720p field exceeds the maximum length: %d > %d".formatted(data.getVideo720p().size(), 100));

      if(data.getVideo360p().size() <= 100)
      cdr.write_type_e(data.getVideo360p());else
          throw new RuntimeException("video360p field exceeds the maximum length: %d > %d".formatted(data.getVideo360p().size(), 100));

      if(data.getVideo180p().size() <= 100)
      cdr.write_type_e(data.getVideo180p());else
          throw new RuntimeException("video180p field exceeds the maximum length: %d > %d".formatted(data.getVideo180p().size(), 100));

   }

   public static void read(unitree_go_msgs.msg.dds.Go2FrontVideoData data, us.ihmc.idl.CDR cdr)
   {
      data.setTimeFrame(cdr.read_type_12());
      	
      cdr.read_type_e(data.getVideo720p());	
      cdr.read_type_e(data.getVideo360p());	
      cdr.read_type_e(data.getVideo180p());	

   }

   @Override
   public final void serialize(unitree_go_msgs.msg.dds.Go2FrontVideoData data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_12("time_frame", data.getTimeFrame());
      ser.write_type_e("video720p", data.getVideo720p());
      ser.write_type_e("video360p", data.getVideo360p());
      ser.write_type_e("video180p", data.getVideo180p());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, unitree_go_msgs.msg.dds.Go2FrontVideoData data)
   {
      data.setTimeFrame(ser.read_type_12("time_frame"));
      ser.read_type_e("video720p", data.getVideo720p());
      ser.read_type_e("video360p", data.getVideo360p());
      ser.read_type_e("video180p", data.getVideo180p());
   }

   public static void staticCopy(unitree_go_msgs.msg.dds.Go2FrontVideoData src, unitree_go_msgs.msg.dds.Go2FrontVideoData dest)
   {
      dest.set(src);
   }

   @Override
   public unitree_go_msgs.msg.dds.Go2FrontVideoData createData()
   {
      return new unitree_go_msgs.msg.dds.Go2FrontVideoData();
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
   
   public void serialize(unitree_go_msgs.msg.dds.Go2FrontVideoData data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(unitree_go_msgs.msg.dds.Go2FrontVideoData data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(unitree_go_msgs.msg.dds.Go2FrontVideoData src, unitree_go_msgs.msg.dds.Go2FrontVideoData dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public Go2FrontVideoDataPubSubType newInstance()
   {
      return new Go2FrontVideoDataPubSubType();
   }
}
