package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "RobotFrameData" defined in "RobotFrameData_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from RobotFrameData_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit RobotFrameData_.idl instead.
*
*/
public class RobotFrameDataPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.RobotFrameData>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::RobotFrameData_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.RobotFrameData data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.RobotFrameData data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 2056; ++i0)
      {
        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      }

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.RobotFrameData data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.RobotFrameData data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getFramePoseInWorld(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getFrameName().size(); ++i0)
      {
          current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getFrameName().get(i0).length() + 1;
      }

      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.RobotFrameData data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_11(data.getTimestamp());

      cdr.write_type_2(data.getFrameNameHash());

      geometry_msgs.msg.dds.PosePubSubType.write(data.getFramePoseInWorld(), cdr);
      if(data.getFrameName().size() <= 2056)
      cdr.write_type_e(data.getFrameName());else
          throw new RuntimeException("frame_name field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.RobotFrameData data, us.ihmc.idl.CDR cdr)
   {
      data.setTimestamp(cdr.read_type_11());
      	
      data.setFrameNameHash(cdr.read_type_2());
      	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getFramePoseInWorld(), cdr);	
      cdr.read_type_e(data.getFrameName());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.RobotFrameData data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_11("timestamp", data.getTimestamp());
      ser.write_type_2("frame_name_hash", data.getFrameNameHash());
      ser.write_type_a("frame_pose_in_world", new geometry_msgs.msg.dds.PosePubSubType(), data.getFramePoseInWorld());

      ser.write_type_e("frame_name", data.getFrameName());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.RobotFrameData data)
   {
      data.setTimestamp(ser.read_type_11("timestamp"));
      data.setFrameNameHash(ser.read_type_2("frame_name_hash"));
      ser.read_type_a("frame_pose_in_world", new geometry_msgs.msg.dds.PosePubSubType(), data.getFramePoseInWorld());

      ser.read_type_e("frame_name", data.getFrameName());
   }

   public static void staticCopy(controller_msgs.msg.dds.RobotFrameData src, controller_msgs.msg.dds.RobotFrameData dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.RobotFrameData createData()
   {
      return new controller_msgs.msg.dds.RobotFrameData();
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
   
   public void serialize(controller_msgs.msg.dds.RobotFrameData data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.RobotFrameData data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.RobotFrameData src, controller_msgs.msg.dds.RobotFrameData dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public RobotFrameDataPubSubType newInstance()
   {
      return new RobotFrameDataPubSubType();
   }
}
