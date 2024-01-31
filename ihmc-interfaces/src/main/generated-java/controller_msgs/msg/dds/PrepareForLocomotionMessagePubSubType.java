package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PrepareForLocomotionMessage" defined in "PrepareForLocomotionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PrepareForLocomotionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PrepareForLocomotionMessage_.idl instead.
*
*/
public class PrepareForLocomotionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.PrepareForLocomotionMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::PrepareForLocomotionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "b72181e318ebba9e1f22e1ebe710a8468da4f23e5539ec11f3778b340afcaa5c";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.PrepareForLocomotionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.PrepareForLocomotionMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PrepareForLocomotionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PrepareForLocomotionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.PrepareForLocomotionMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getPrepareManipulation());

      cdr.write_type_7(data.getPrepareChest());

      cdr.write_type_7(data.getPreparePelvis());

   }

   public static void read(controller_msgs.msg.dds.PrepareForLocomotionMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setPrepareManipulation(cdr.read_type_7());
      	
      data.setPrepareChest(cdr.read_type_7());
      	
      data.setPreparePelvis(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.PrepareForLocomotionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("prepare_manipulation", data.getPrepareManipulation());
      ser.write_type_7("prepare_chest", data.getPrepareChest());
      ser.write_type_7("prepare_pelvis", data.getPreparePelvis());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.PrepareForLocomotionMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setPrepareManipulation(ser.read_type_7("prepare_manipulation"));
      data.setPrepareChest(ser.read_type_7("prepare_chest"));
      data.setPreparePelvis(ser.read_type_7("prepare_pelvis"));
   }

   public static void staticCopy(controller_msgs.msg.dds.PrepareForLocomotionMessage src, controller_msgs.msg.dds.PrepareForLocomotionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.PrepareForLocomotionMessage createData()
   {
      return new controller_msgs.msg.dds.PrepareForLocomotionMessage();
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
   
   public void serialize(controller_msgs.msg.dds.PrepareForLocomotionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.PrepareForLocomotionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.PrepareForLocomotionMessage src, controller_msgs.msg.dds.PrepareForLocomotionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PrepareForLocomotionMessagePubSubType newInstance()
   {
      return new PrepareForLocomotionMessagePubSubType();
   }
}
