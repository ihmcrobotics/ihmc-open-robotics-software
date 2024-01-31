package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "OneDoFJointTrajectoryMessage" defined in "OneDoFJointTrajectoryMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from OneDoFJointTrajectoryMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit OneDoFJointTrajectoryMessage_.idl instead.
*
*/
public class OneDoFJointTrajectoryMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.OneDoFJointTrajectoryMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::OneDoFJointTrajectoryMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "906a241c529d5f4d4e0a47ba01e65d50152118ec79f67fca45823b510a60e2d8";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.OneDoFJointTrajectoryMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.OneDoFJointTrajectoryMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 50; ++i0)
      {
          current_alignment += ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.OneDoFJointTrajectoryMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.OneDoFJointTrajectoryMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getTrajectoryPoints().size(); ++i0)
      {
          current_alignment += ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessagePubSubType.getCdrSerializedSize(data.getTrajectoryPoints().get(i0), current_alignment);}

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.OneDoFJointTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getTrajectoryPoints().size() <= 50)
      cdr.write_type_e(data.getTrajectoryPoints());else
          throw new RuntimeException("trajectory_points field exceeds the maximum length");

      cdr.write_type_6(data.getWeight());

   }

   public static void read(controller_msgs.msg.dds.OneDoFJointTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getTrajectoryPoints());	
      data.setWeight(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.OneDoFJointTrajectoryMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_e("trajectory_points", data.getTrajectoryPoints());
      ser.write_type_6("weight", data.getWeight());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.OneDoFJointTrajectoryMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_e("trajectory_points", data.getTrajectoryPoints());
      data.setWeight(ser.read_type_6("weight"));
   }

   public static void staticCopy(controller_msgs.msg.dds.OneDoFJointTrajectoryMessage src, controller_msgs.msg.dds.OneDoFJointTrajectoryMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.OneDoFJointTrajectoryMessage createData()
   {
      return new controller_msgs.msg.dds.OneDoFJointTrajectoryMessage();
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
   
   public void serialize(controller_msgs.msg.dds.OneDoFJointTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.OneDoFJointTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.OneDoFJointTrajectoryMessage src, controller_msgs.msg.dds.OneDoFJointTrajectoryMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public OneDoFJointTrajectoryMessagePubSubType newInstance()
   {
      return new OneDoFJointTrajectoryMessagePubSubType();
   }
}
