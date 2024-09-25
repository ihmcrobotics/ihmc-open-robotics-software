package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ObjectCarryMessage" defined in "ObjectCarryMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ObjectCarryMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ObjectCarryMessage_.idl instead.
*
*/
public class ObjectCarryMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ObjectCarryMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ObjectCarryMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "ffb4e563bfe57ac26f669c9c72ae26ae2c3450027976335e666392acc9f2f6a1";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ObjectCarryMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ObjectCarryMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ObjectCarryMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ObjectCarryMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ObjectCarryMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getRobotSide());

      cdr.write_type_7(data.getIsPickingUp());

      cdr.write_type_6(data.getObjectMass());

   }

   public static void read(controller_msgs.msg.dds.ObjectCarryMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setRobotSide(cdr.read_type_9());
      	
      data.setIsPickingUp(cdr.read_type_7());
      	
      data.setObjectMass(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ObjectCarryMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_7("is_picking_up", data.getIsPickingUp());
      ser.write_type_6("object_mass", data.getObjectMass());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ObjectCarryMessage data)
   {
      data.setRobotSide(ser.read_type_9("robot_side"));
      data.setIsPickingUp(ser.read_type_7("is_picking_up"));
      data.setObjectMass(ser.read_type_6("object_mass"));
   }

   public static void staticCopy(controller_msgs.msg.dds.ObjectCarryMessage src, controller_msgs.msg.dds.ObjectCarryMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ObjectCarryMessage createData()
   {
      return new controller_msgs.msg.dds.ObjectCarryMessage();
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
   
   public void serialize(controller_msgs.msg.dds.ObjectCarryMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ObjectCarryMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ObjectCarryMessage src, controller_msgs.msg.dds.ObjectCarryMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ObjectCarryMessagePubSubType newInstance()
   {
      return new ObjectCarryMessagePubSubType();
   }
}
