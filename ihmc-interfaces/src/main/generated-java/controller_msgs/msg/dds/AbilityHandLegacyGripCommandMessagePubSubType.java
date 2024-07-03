package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "AbilityHandLegacyGripCommandMessage" defined in "AbilityHandLegacyGripCommandMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from AbilityHandLegacyGripCommandMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit AbilityHandLegacyGripCommandMessage_.idl instead.
*
*/
public class AbilityHandLegacyGripCommandMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.AbilityHandLegacyGripCommandMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::AbilityHandLegacyGripCommandMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "318816d026b71a4ef99ca776c3cfb0a184b3f7aa087060638cbe8a45c04c8d73";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.AbilityHandLegacyGripCommandMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.AbilityHandLegacyGripCommandMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.AbilityHandLegacyGripCommandMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.AbilityHandLegacyGripCommandMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getLegacyGripSpeed().length() + 1;


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.AbilityHandLegacyGripCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getLegacyGripType());

      if(data.getLegacyGripSpeed().length() <= 255)
      cdr.write_type_d(data.getLegacyGripSpeed());else
          throw new RuntimeException("legacy_grip_speed field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.AbilityHandLegacyGripCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setLegacyGripType(cdr.read_type_9());
      	
      cdr.read_type_d(data.getLegacyGripSpeed());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.AbilityHandLegacyGripCommandMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("legacy_grip_type", data.getLegacyGripType());
      ser.write_type_d("legacy_grip_speed", data.getLegacyGripSpeed());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.AbilityHandLegacyGripCommandMessage data)
   {
      data.setLegacyGripType(ser.read_type_9("legacy_grip_type"));
      ser.read_type_d("legacy_grip_speed", data.getLegacyGripSpeed());
   }

   public static void staticCopy(controller_msgs.msg.dds.AbilityHandLegacyGripCommandMessage src, controller_msgs.msg.dds.AbilityHandLegacyGripCommandMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.AbilityHandLegacyGripCommandMessage createData()
   {
      return new controller_msgs.msg.dds.AbilityHandLegacyGripCommandMessage();
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
   
   public void serialize(controller_msgs.msg.dds.AbilityHandLegacyGripCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.AbilityHandLegacyGripCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.AbilityHandLegacyGripCommandMessage src, controller_msgs.msg.dds.AbilityHandLegacyGripCommandMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public AbilityHandLegacyGripCommandMessagePubSubType newInstance()
   {
      return new AbilityHandLegacyGripCommandMessagePubSubType();
   }
}
