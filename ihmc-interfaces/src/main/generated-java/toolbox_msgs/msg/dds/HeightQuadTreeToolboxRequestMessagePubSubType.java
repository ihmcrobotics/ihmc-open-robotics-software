package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HeightQuadTreeToolboxRequestMessage" defined in "HeightQuadTreeToolboxRequestMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HeightQuadTreeToolboxRequestMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HeightQuadTreeToolboxRequestMessage_.idl instead.
*
*/
public class HeightQuadTreeToolboxRequestMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.HeightQuadTreeToolboxRequestMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::HeightQuadTreeToolboxRequestMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "8f336e2f0f4276a4e7d90f7a276ee8d5dd41ce957784b83a0c4f096047fced47";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.HeightQuadTreeToolboxRequestMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.HeightQuadTreeToolboxRequestMessage data) throws java.io.IOException
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


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.HeightQuadTreeToolboxRequestMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.HeightQuadTreeToolboxRequestMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.HeightQuadTreeToolboxRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getRequestClearQuadTree());

      cdr.write_type_7(data.getRequestQuadTreeUpdate());

   }

   public static void read(toolbox_msgs.msg.dds.HeightQuadTreeToolboxRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRequestClearQuadTree(cdr.read_type_7());
      	
      data.setRequestQuadTreeUpdate(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.HeightQuadTreeToolboxRequestMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("request_clear_quad_tree", data.getRequestClearQuadTree());
      ser.write_type_7("request_quad_tree_update", data.getRequestQuadTreeUpdate());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.HeightQuadTreeToolboxRequestMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRequestClearQuadTree(ser.read_type_7("request_clear_quad_tree"));
      data.setRequestQuadTreeUpdate(ser.read_type_7("request_quad_tree_update"));
   }

   public static void staticCopy(toolbox_msgs.msg.dds.HeightQuadTreeToolboxRequestMessage src, toolbox_msgs.msg.dds.HeightQuadTreeToolboxRequestMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.HeightQuadTreeToolboxRequestMessage createData()
   {
      return new toolbox_msgs.msg.dds.HeightQuadTreeToolboxRequestMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.HeightQuadTreeToolboxRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.HeightQuadTreeToolboxRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.HeightQuadTreeToolboxRequestMessage src, toolbox_msgs.msg.dds.HeightQuadTreeToolboxRequestMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HeightQuadTreeToolboxRequestMessagePubSubType newInstance()
   {
      return new HeightQuadTreeToolboxRequestMessagePubSubType();
   }
}
