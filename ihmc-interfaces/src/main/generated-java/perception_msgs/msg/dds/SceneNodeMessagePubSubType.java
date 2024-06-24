package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SceneNodeMessage" defined in "SceneNodeMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SceneNodeMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SceneNodeMessage_.idl instead.
*
*/
public class SceneNodeMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.SceneNodeMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::SceneNodeMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "ca25a45e13424ef0f85ebeea7bb407ce74c28e53277cc06634d73ff5a901095a";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.SceneNodeMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.SceneNodeMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += ihmc_common_msgs.msg.dds.ConfirmableRequestMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.SceneNodeMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.SceneNodeMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getName().length() + 1;

      current_alignment += ihmc_common_msgs.msg.dds.ConfirmableRequestMessagePubSubType.getCdrSerializedSize(data.getConfirmableRequest(), current_alignment);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getTransformToWorld(), current_alignment);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.SceneNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getId());

      if(data.getName().length() <= 255)
      cdr.write_type_d(data.getName());else
          throw new RuntimeException("name field exceeds the maximum length");

      ihmc_common_msgs.msg.dds.ConfirmableRequestMessagePubSubType.write(data.getConfirmableRequest(), cdr);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getTransformToWorld(), cdr);
      cdr.write_type_3(data.getNumberOfChildren());

   }

   public static void read(perception_msgs.msg.dds.SceneNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setId(cdr.read_type_4());
      	
      cdr.read_type_d(data.getName());	
      ihmc_common_msgs.msg.dds.ConfirmableRequestMessagePubSubType.read(data.getConfirmableRequest(), cdr);	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getTransformToWorld(), cdr);	
      data.setNumberOfChildren(cdr.read_type_3());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.SceneNodeMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("id", data.getId());
      ser.write_type_d("name", data.getName());
      ser.write_type_a("confirmable_request", new ihmc_common_msgs.msg.dds.ConfirmableRequestMessagePubSubType(), data.getConfirmableRequest());

      ser.write_type_a("transform_to_world", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getTransformToWorld());

      ser.write_type_3("number_of_children", data.getNumberOfChildren());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.SceneNodeMessage data)
   {
      data.setId(ser.read_type_4("id"));
      ser.read_type_d("name", data.getName());
      ser.read_type_a("confirmable_request", new ihmc_common_msgs.msg.dds.ConfirmableRequestMessagePubSubType(), data.getConfirmableRequest());

      ser.read_type_a("transform_to_world", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getTransformToWorld());

      data.setNumberOfChildren(ser.read_type_3("number_of_children"));
   }

   public static void staticCopy(perception_msgs.msg.dds.SceneNodeMessage src, perception_msgs.msg.dds.SceneNodeMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.SceneNodeMessage createData()
   {
      return new perception_msgs.msg.dds.SceneNodeMessage();
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
   
   public void serialize(perception_msgs.msg.dds.SceneNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.SceneNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.SceneNodeMessage src, perception_msgs.msg.dds.SceneNodeMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SceneNodeMessagePubSubType newInstance()
   {
      return new SceneNodeMessagePubSubType();
   }
}
