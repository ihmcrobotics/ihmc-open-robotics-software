package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "DirectionalControlConfigurationMessage" defined in "DirectionalControlConfigurationMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from DirectionalControlConfigurationMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit DirectionalControlConfigurationMessage_.idl instead.
*
*/
public class DirectionalControlConfigurationMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.DirectionalControlConfigurationMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::DirectionalControlConfigurationMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "2dbfb6aaf460a3c9a023793ef4044afe15188dcdf5c18ac6bcbebb630d9a3e57";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.DirectionalControlConfigurationMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.DirectionalControlConfigurationMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.DirectionalControlConfigurationMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.DirectionalControlConfigurationMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getProfileName().length() + 1;


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.DirectionalControlConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getEnableWalking());

      if(data.getProfileName().length() <= 255)
      cdr.write_type_d(data.getProfileName());else
          throw new RuntimeException("profile_name field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.DirectionalControlConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setEnableWalking(cdr.read_type_7());
      	
      cdr.read_type_d(data.getProfileName());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.DirectionalControlConfigurationMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("enable_walking", data.getEnableWalking());
      ser.write_type_d("profile_name", data.getProfileName());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.DirectionalControlConfigurationMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setEnableWalking(ser.read_type_7("enable_walking"));
      ser.read_type_d("profile_name", data.getProfileName());
   }

   public static void staticCopy(controller_msgs.msg.dds.DirectionalControlConfigurationMessage src, controller_msgs.msg.dds.DirectionalControlConfigurationMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.DirectionalControlConfigurationMessage createData()
   {
      return new controller_msgs.msg.dds.DirectionalControlConfigurationMessage();
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
   
   public void serialize(controller_msgs.msg.dds.DirectionalControlConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.DirectionalControlConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.DirectionalControlConfigurationMessage src, controller_msgs.msg.dds.DirectionalControlConfigurationMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public DirectionalControlConfigurationMessagePubSubType newInstance()
   {
      return new DirectionalControlConfigurationMessagePubSubType();
   }
}
