package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "JoystickRemoteInputMessage" defined in "JoystickRemoteInputMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from JoystickRemoteInputMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit JoystickRemoteInputMessage_.idl instead.
*
*/
public class JoystickRemoteInputMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.JoystickRemoteInputMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::JoystickRemoteInputMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.JoystickRemoteInputMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.JoystickRemoteInputMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.JoystickRemoteInputMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.JoystickRemoteInputMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.JoystickRemoteInputMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_6(data.getForward());

      cdr.write_type_6(data.getRight());

      cdr.write_type_6(data.getClockwise());

   }

   public static void read(controller_msgs.msg.dds.JoystickRemoteInputMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setForward(cdr.read_type_6());
      	
      data.setRight(cdr.read_type_6());
      	
      data.setClockwise(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.JoystickRemoteInputMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_6("forward", data.getForward());
      ser.write_type_6("right", data.getRight());
      ser.write_type_6("clockwise", data.getClockwise());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.JoystickRemoteInputMessage data)
   {
      data.setForward(ser.read_type_6("forward"));
      data.setRight(ser.read_type_6("right"));
      data.setClockwise(ser.read_type_6("clockwise"));
   }

   public static void staticCopy(controller_msgs.msg.dds.JoystickRemoteInputMessage src, controller_msgs.msg.dds.JoystickRemoteInputMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.JoystickRemoteInputMessage createData()
   {
      return new controller_msgs.msg.dds.JoystickRemoteInputMessage();
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
   
   public void serialize(controller_msgs.msg.dds.JoystickRemoteInputMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.JoystickRemoteInputMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.JoystickRemoteInputMessage src, controller_msgs.msg.dds.JoystickRemoteInputMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public JoystickRemoteInputMessagePubSubType newInstance()
   {
      return new JoystickRemoteInputMessagePubSubType();
   }
}
