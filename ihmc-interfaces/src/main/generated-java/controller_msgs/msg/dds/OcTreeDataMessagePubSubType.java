package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "OcTreeDataMessage" defined in "OcTreeDataMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from OcTreeDataMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit OcTreeDataMessage_.idl instead.
*
*/
public class OcTreeDataMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.OcTreeDataMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::OcTreeDataMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.OcTreeDataMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.OcTreeDataMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (5000 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (5000 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (5000 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (5000 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.OcTreeDataMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.OcTreeDataMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getXKeys().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getYKeys().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getZKeys().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getTreeDepth().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.OcTreeDataMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_2(data.getMaximumTreeDepth());

      cdr.write_type_6(data.getTreeResolution());

      if(data.getXKeys().size() <= 5000)
      cdr.write_type_e(data.getXKeys());else
          throw new RuntimeException("x_keys field exceeds the maximum length");

      if(data.getYKeys().size() <= 5000)
      cdr.write_type_e(data.getYKeys());else
          throw new RuntimeException("y_keys field exceeds the maximum length");

      if(data.getZKeys().size() <= 5000)
      cdr.write_type_e(data.getZKeys());else
          throw new RuntimeException("z_keys field exceeds the maximum length");

      if(data.getTreeDepth().size() <= 5000)
      cdr.write_type_e(data.getTreeDepth());else
          throw new RuntimeException("tree_depth field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.OcTreeDataMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setMaximumTreeDepth(cdr.read_type_2());
      	
      data.setTreeResolution(cdr.read_type_6());
      	
      cdr.read_type_e(data.getXKeys());	
      cdr.read_type_e(data.getYKeys());	
      cdr.read_type_e(data.getZKeys());	
      cdr.read_type_e(data.getTreeDepth());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.OcTreeDataMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_2("maximum_tree_depth", data.getMaximumTreeDepth());
      ser.write_type_6("tree_resolution", data.getTreeResolution());
      ser.write_type_e("x_keys", data.getXKeys());
      ser.write_type_e("y_keys", data.getYKeys());
      ser.write_type_e("z_keys", data.getZKeys());
      ser.write_type_e("tree_depth", data.getTreeDepth());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.OcTreeDataMessage data)
   {
      data.setMaximumTreeDepth(ser.read_type_2("maximum_tree_depth"));
      data.setTreeResolution(ser.read_type_6("tree_resolution"));
      ser.read_type_e("x_keys", data.getXKeys());
      ser.read_type_e("y_keys", data.getYKeys());
      ser.read_type_e("z_keys", data.getZKeys());
      ser.read_type_e("tree_depth", data.getTreeDepth());
   }

   public static void staticCopy(controller_msgs.msg.dds.OcTreeDataMessage src, controller_msgs.msg.dds.OcTreeDataMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.OcTreeDataMessage createData()
   {
      return new controller_msgs.msg.dds.OcTreeDataMessage();
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
   
   public void serialize(controller_msgs.msg.dds.OcTreeDataMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.OcTreeDataMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.OcTreeDataMessage src, controller_msgs.msg.dds.OcTreeDataMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public OcTreeDataMessagePubSubType newInstance()
   {
      return new OcTreeDataMessagePubSubType();
   }
}
