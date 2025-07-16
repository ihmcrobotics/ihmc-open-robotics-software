package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "MasterGainScaleControllerCommandMessage" defined in "MasterGainScaleControllerCommandMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from MasterGainScaleControllerCommandMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit MasterGainScaleControllerCommandMessage_.idl instead.
*
*/
public class MasterGainScaleControllerCommandMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.MasterGainScaleControllerCommandMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::MasterGainScaleControllerCommandMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "d844ff29ec3d4f09434fd204eb3958036a2e1b6f290ef7993f4db2c69cb39788";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.MasterGainScaleControllerCommandMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.MasterGainScaleControllerCommandMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.MasterGainScaleControllerCommandMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.MasterGainScaleControllerCommandMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.MasterGainScaleControllerCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_7(data.getServoRobot());

      cdr.write_type_7(data.getUnservoImmediately());

      cdr.write_type_7(data.getUnservoSlowly());

   }

   public static void read(controller_msgs.msg.dds.MasterGainScaleControllerCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setServoRobot(cdr.read_type_7());
      	
      data.setUnservoImmediately(cdr.read_type_7());
      	
      data.setUnservoSlowly(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.MasterGainScaleControllerCommandMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("servo_robot", data.getServoRobot());
      ser.write_type_7("unservo_immediately", data.getUnservoImmediately());
      ser.write_type_7("unservo_slowly", data.getUnservoSlowly());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.MasterGainScaleControllerCommandMessage data)
   {
      data.setServoRobot(ser.read_type_7("servo_robot"));
      data.setUnservoImmediately(ser.read_type_7("unservo_immediately"));
      data.setUnservoSlowly(ser.read_type_7("unservo_slowly"));
   }

   public static void staticCopy(controller_msgs.msg.dds.MasterGainScaleControllerCommandMessage src, controller_msgs.msg.dds.MasterGainScaleControllerCommandMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.MasterGainScaleControllerCommandMessage createData()
   {
      return new controller_msgs.msg.dds.MasterGainScaleControllerCommandMessage();
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
   
   public void serialize(controller_msgs.msg.dds.MasterGainScaleControllerCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.MasterGainScaleControllerCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.MasterGainScaleControllerCommandMessage src, controller_msgs.msg.dds.MasterGainScaleControllerCommandMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public MasterGainScaleControllerCommandMessagePubSubType newInstance()
   {
      return new MasterGainScaleControllerCommandMessagePubSubType();
   }
}
