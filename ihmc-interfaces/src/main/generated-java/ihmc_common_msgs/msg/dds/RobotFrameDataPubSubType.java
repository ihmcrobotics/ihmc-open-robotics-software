package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "RobotFrameData" defined in "RobotFrameData_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from RobotFrameData_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit RobotFrameData_.idl instead.
*
*/
public class RobotFrameDataPubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.RobotFrameData>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::RobotFrameData_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "45991a519226e00fa20fc0477f2d5e631fd66f1fde594c0194273f2685782939";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.RobotFrameData data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.RobotFrameData data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.RobotFrameData data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.RobotFrameData data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getFramePoseInWorld(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getFrameName().length() + 1;


      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.RobotFrameData data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_11(data.getTimestamp());

      cdr.write_type_2(data.getFrameNameHash());

      geometry_msgs.msg.dds.PosePubSubType.write(data.getFramePoseInWorld(), cdr);
      if(data.getFrameName().length() <= 255)
      cdr.write_type_d(data.getFrameName());else
          throw new RuntimeException("frame_name field exceeds the maximum length");

   }

   public static void read(ihmc_common_msgs.msg.dds.RobotFrameData data, us.ihmc.idl.CDR cdr)
   {
      data.setTimestamp(cdr.read_type_11());
      	
      data.setFrameNameHash(cdr.read_type_2());
      	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getFramePoseInWorld(), cdr);	
      cdr.read_type_d(data.getFrameName());	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.RobotFrameData data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_11("timestamp", data.getTimestamp());
      ser.write_type_2("frame_name_hash", data.getFrameNameHash());
      ser.write_type_a("frame_pose_in_world", new geometry_msgs.msg.dds.PosePubSubType(), data.getFramePoseInWorld());

      ser.write_type_d("frame_name", data.getFrameName());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.RobotFrameData data)
   {
      data.setTimestamp(ser.read_type_11("timestamp"));
      data.setFrameNameHash(ser.read_type_2("frame_name_hash"));
      ser.read_type_a("frame_pose_in_world", new geometry_msgs.msg.dds.PosePubSubType(), data.getFramePoseInWorld());

      ser.read_type_d("frame_name", data.getFrameName());
   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.RobotFrameData src, ihmc_common_msgs.msg.dds.RobotFrameData dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.RobotFrameData createData()
   {
      return new ihmc_common_msgs.msg.dds.RobotFrameData();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.RobotFrameData data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.RobotFrameData data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.RobotFrameData src, ihmc_common_msgs.msg.dds.RobotFrameData dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public RobotFrameDataPubSubType newInstance()
   {
      return new RobotFrameDataPubSubType();
   }
}
