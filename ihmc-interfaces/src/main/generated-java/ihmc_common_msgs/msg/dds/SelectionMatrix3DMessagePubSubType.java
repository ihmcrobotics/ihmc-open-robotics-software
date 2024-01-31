package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SelectionMatrix3DMessage" defined in "SelectionMatrix3DMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SelectionMatrix3DMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SelectionMatrix3DMessage_.idl instead.
*
*/
public class SelectionMatrix3DMessagePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.SelectionMatrix3DMessage>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::SelectionMatrix3DMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "cf0985877b2af50c77b4e1b09a85cc74d197459e19ef1707ee3466050f437661";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.SelectionMatrix3DMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.SelectionMatrix3DMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.SelectionMatrix3DMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.SelectionMatrix3DMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.SelectionMatrix3DMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_11(data.getSelectionFrameId());

      cdr.write_type_7(data.getXSelected());

      cdr.write_type_7(data.getYSelected());

      cdr.write_type_7(data.getZSelected());

   }

   public static void read(ihmc_common_msgs.msg.dds.SelectionMatrix3DMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setSelectionFrameId(cdr.read_type_11());
      	
      data.setXSelected(cdr.read_type_7());
      	
      data.setYSelected(cdr.read_type_7());
      	
      data.setZSelected(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.SelectionMatrix3DMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_11("selection_frame_id", data.getSelectionFrameId());
      ser.write_type_7("x_selected", data.getXSelected());
      ser.write_type_7("y_selected", data.getYSelected());
      ser.write_type_7("z_selected", data.getZSelected());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.SelectionMatrix3DMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setSelectionFrameId(ser.read_type_11("selection_frame_id"));
      data.setXSelected(ser.read_type_7("x_selected"));
      data.setYSelected(ser.read_type_7("y_selected"));
      data.setZSelected(ser.read_type_7("z_selected"));
   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.SelectionMatrix3DMessage src, ihmc_common_msgs.msg.dds.SelectionMatrix3DMessage dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.SelectionMatrix3DMessage createData()
   {
      return new ihmc_common_msgs.msg.dds.SelectionMatrix3DMessage();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.SelectionMatrix3DMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.SelectionMatrix3DMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.SelectionMatrix3DMessage src, ihmc_common_msgs.msg.dds.SelectionMatrix3DMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SelectionMatrix3DMessagePubSubType newInstance()
   {
      return new SelectionMatrix3DMessagePubSubType();
   }
}
