package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ScrewPrimitiveActionStateMessage" defined in "ScrewPrimitiveActionStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ScrewPrimitiveActionStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ScrewPrimitiveActionStateMessage_.idl instead.
*
*/
public class ScrewPrimitiveActionStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::ScrewPrimitiveActionStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "5a2a5d47079afdc07173d70d83dc0036e78b1f1c0f1486abb36b1ed00d66e665";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 50; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.getCdrSerializedSize(data.getState(), current_alignment);

      current_alignment += behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getTrajectory().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getTrajectory().get(i0), current_alignment);}

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getForce(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getTorque(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.write(data.getState(), cdr);
      behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      if(data.getTrajectory().size() <= 50)
      cdr.write_type_e(data.getTrajectory());else
          throw new RuntimeException("trajectory field exceeds the maximum length");

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getForce(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getTorque(), cdr);
   }

   public static void read(behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.read(data.getState(), cdr);	
      behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      cdr.read_type_e(data.getTrajectory());	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getForce(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getTorque(), cdr);	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("state", new behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType(), data.getState());

      ser.write_type_a("definition", new behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_e("trajectory", data.getTrajectory());
      ser.write_type_a("force", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getForce());

      ser.write_type_a("torque", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getTorque());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage data)
   {
      ser.read_type_a("state", new behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType(), data.getState());

      ser.read_type_a("definition", new behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessagePubSubType(), data.getDefinition());

      ser.read_type_e("trajectory", data.getTrajectory());
      ser.read_type_a("force", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getForce());

      ser.read_type_a("torque", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getTorque());

   }

   public static void staticCopy(behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage src, behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage createData()
   {
      return new behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage src, behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ScrewPrimitiveActionStateMessagePubSubType newInstance()
   {
      return new ScrewPrimitiveActionStateMessagePubSubType();
   }
}
