package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "AbilityHandStatusMessage" defined in "AbilityHandStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from AbilityHandStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit AbilityHandStatusMessage_.idl instead.
*
*/
public class AbilityHandStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.AbilityHandStatusMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::AbilityHandStatusMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "502cf8ef141a8e538c74ab94797a3c5662e400eb52a98fa47415ffc6337cccd5";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.AbilityHandStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.AbilityHandStatusMessage data) throws java.io.IOException
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

      current_alignment += ((6) * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.AbilityHandStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.AbilityHandStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += ((6) * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.AbilityHandStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getRobotSide());

      for(int i0 = 0; i0 < data.getFingerPositions().length; ++i0)
      {
        	cdr.write_type_6(data.getFingerPositions()[i0]);	
      }

   }

   public static void read(controller_msgs.msg.dds.AbilityHandStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setRobotSide(cdr.read_type_9());
      	
      for(int i0 = 0; i0 < data.getFingerPositions().length; ++i0)
      {
        	data.getFingerPositions()[i0] = cdr.read_type_6();
        	
      }
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.AbilityHandStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_f("finger_positions", data.getFingerPositions());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.AbilityHandStatusMessage data)
   {
      data.setRobotSide(ser.read_type_9("robot_side"));
      ser.read_type_f("finger_positions", data.getFingerPositions());
   }

   public static void staticCopy(controller_msgs.msg.dds.AbilityHandStatusMessage src, controller_msgs.msg.dds.AbilityHandStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.AbilityHandStatusMessage createData()
   {
      return new controller_msgs.msg.dds.AbilityHandStatusMessage();
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
   
   public void serialize(controller_msgs.msg.dds.AbilityHandStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.AbilityHandStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.AbilityHandStatusMessage src, controller_msgs.msg.dds.AbilityHandStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public AbilityHandStatusMessagePubSubType newInstance()
   {
      return new AbilityHandStatusMessagePubSubType();
   }
}
