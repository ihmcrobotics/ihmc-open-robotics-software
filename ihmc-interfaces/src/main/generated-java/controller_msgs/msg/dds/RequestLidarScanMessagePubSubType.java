package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "RequestLidarScanMessage" defined in "RequestLidarScanMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from RequestLidarScanMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit RequestLidarScanMessage_.idl instead.
*
*/
public class RequestLidarScanMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.RequestLidarScanMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::RequestLidarScanMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.RequestLidarScanMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.RequestLidarScanMessage data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.RequestLidarScanMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.RequestLidarScanMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.RequestLidarScanMessage data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_4(data.getSequenceId());


      cdr.write_type_7(data.getRemoveShadows());


      cdr.write_type_7(data.getRemoveSelfCollisions());

   }

   public static void read(controller_msgs.msg.dds.RequestLidarScanMessage data, us.ihmc.idl.CDR cdr)
   {

      data.setSequenceId(cdr.read_type_4());
      	

      data.setRemoveShadows(cdr.read_type_7());
      	

      data.setRemoveSelfCollisions(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.RequestLidarScanMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {

      ser.write_type_4("sequence_id", data.getSequenceId());

      ser.write_type_7("remove_shadows", data.getRemoveShadows());

      ser.write_type_7("remove_self_collisions", data.getRemoveSelfCollisions());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.RequestLidarScanMessage data)
   {

      data.setSequenceId(ser.read_type_4("sequence_id"));

      data.setRemoveShadows(ser.read_type_7("remove_shadows"));

      data.setRemoveSelfCollisions(ser.read_type_7("remove_self_collisions"));
   }

   public static void staticCopy(controller_msgs.msg.dds.RequestLidarScanMessage src, controller_msgs.msg.dds.RequestLidarScanMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.RequestLidarScanMessage createData()
   {
      return new controller_msgs.msg.dds.RequestLidarScanMessage();
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
   
   public void serialize(controller_msgs.msg.dds.RequestLidarScanMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.RequestLidarScanMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.RequestLidarScanMessage src, controller_msgs.msg.dds.RequestLidarScanMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public RequestLidarScanMessagePubSubType newInstance()
   {
      return new RequestLidarScanMessagePubSubType();
   }
}
