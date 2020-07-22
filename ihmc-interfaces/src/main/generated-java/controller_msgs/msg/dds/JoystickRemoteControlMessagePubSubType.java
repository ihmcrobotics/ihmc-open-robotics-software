package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "JoystickRemoteControlMessage" defined in "JoystickRemoteControlMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from JoystickRemoteControlMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit JoystickRemoteControlMessage_.idl instead.
*
*/
public class JoystickRemoteControlMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.JoystickRemoteControlMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::JoystickRemoteControlMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.JoystickRemoteControlMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.JoystickRemoteControlMessage data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.JoystickRemoteControlMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.JoystickRemoteControlMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getProfileName().length() + 1;


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.JoystickRemoteControlMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_7(data.getEnableWalking());

      if(data.getProfileName().length() <= 255)
      cdr.write_type_d(data.getProfileName());else
          throw new RuntimeException("profile_name field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.JoystickRemoteControlMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setEnableWalking(cdr.read_type_7());
      	
      cdr.read_type_d(data.getProfileName());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.JoystickRemoteControlMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("enable_walking", data.getEnableWalking());
      ser.write_type_d("profile_name", data.getProfileName());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.JoystickRemoteControlMessage data)
   {
      data.setEnableWalking(ser.read_type_7("enable_walking"));
      ser.read_type_d("profile_name", data.getProfileName());
   }

   public static void staticCopy(controller_msgs.msg.dds.JoystickRemoteControlMessage src, controller_msgs.msg.dds.JoystickRemoteControlMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.JoystickRemoteControlMessage createData()
   {
      return new controller_msgs.msg.dds.JoystickRemoteControlMessage();
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
   
   public void serialize(controller_msgs.msg.dds.JoystickRemoteControlMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.JoystickRemoteControlMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.JoystickRemoteControlMessage src, controller_msgs.msg.dds.JoystickRemoteControlMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public JoystickRemoteControlMessagePubSubType newInstance()
   {
      return new JoystickRemoteControlMessagePubSubType();
   }
}
