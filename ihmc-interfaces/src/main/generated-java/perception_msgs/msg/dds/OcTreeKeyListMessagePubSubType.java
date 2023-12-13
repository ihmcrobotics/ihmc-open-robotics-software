package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "OcTreeKeyListMessage" defined in "OcTreeKeyListMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from OcTreeKeyListMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit OcTreeKeyListMessage_.idl instead.
*
*/
public class OcTreeKeyListMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.OcTreeKeyListMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::OcTreeKeyListMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "d93f95adbb50d343f76ad11bdedeb73de3bd5dfa8d378eb4dcbf05fd3737bb52";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.OcTreeKeyListMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.OcTreeKeyListMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (200000 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.OcTreeKeyListMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.OcTreeKeyListMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getKeys().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.OcTreeKeyListMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_2(data.getTreeDepth());

      cdr.write_type_6(data.getTreeResolution());

      cdr.write_type_2(data.getNumberOfKeys());

      if(data.getKeys().size() <= 200000)
      cdr.write_type_e(data.getKeys());else
          throw new RuntimeException("keys field exceeds the maximum length");

   }

   public static void read(perception_msgs.msg.dds.OcTreeKeyListMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setTreeDepth(cdr.read_type_2());
      	
      data.setTreeResolution(cdr.read_type_6());
      	
      data.setNumberOfKeys(cdr.read_type_2());
      	
      cdr.read_type_e(data.getKeys());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.OcTreeKeyListMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_2("tree_depth", data.getTreeDepth());
      ser.write_type_6("tree_resolution", data.getTreeResolution());
      ser.write_type_2("number_of_keys", data.getNumberOfKeys());
      ser.write_type_e("keys", data.getKeys());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.OcTreeKeyListMessage data)
   {
      data.setTreeDepth(ser.read_type_2("tree_depth"));
      data.setTreeResolution(ser.read_type_6("tree_resolution"));
      data.setNumberOfKeys(ser.read_type_2("number_of_keys"));
      ser.read_type_e("keys", data.getKeys());
   }

   public static void staticCopy(perception_msgs.msg.dds.OcTreeKeyListMessage src, perception_msgs.msg.dds.OcTreeKeyListMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.OcTreeKeyListMessage createData()
   {
      return new perception_msgs.msg.dds.OcTreeKeyListMessage();
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
   
   public void serialize(perception_msgs.msg.dds.OcTreeKeyListMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.OcTreeKeyListMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.OcTreeKeyListMessage src, perception_msgs.msg.dds.OcTreeKeyListMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public OcTreeKeyListMessagePubSubType newInstance()
   {
      return new OcTreeKeyListMessagePubSubType();
   }
}
