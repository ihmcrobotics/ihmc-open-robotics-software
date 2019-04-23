package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "QuadrupedSupportPlanarRegionParametersMessage" defined in "QuadrupedSupportPlanarRegionParametersMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from QuadrupedSupportPlanarRegionParametersMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit QuadrupedSupportPlanarRegionParametersMessage_.idl instead.
*
*/
public class QuadrupedSupportPlanarRegionParametersMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::QuadrupedSupportPlanarRegionParametersMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_7(data.getEnable());

      cdr.write_type_6(data.getSupportRegionSize());

   }

   public static void read(controller_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setEnable(cdr.read_type_7());
      	
      data.setSupportRegionSize(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("enable", data.getEnable());
      ser.write_type_6("support_region_size", data.getSupportRegionSize());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage data)
   {
      data.setEnable(ser.read_type_7("enable"));
      data.setSupportRegionSize(ser.read_type_6("support_region_size"));
   }

   public static void staticCopy(controller_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage src, controller_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage createData()
   {
      return new controller_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage();
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
   
   public void serialize(controller_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage src, controller_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public QuadrupedSupportPlanarRegionParametersMessagePubSubType newInstance()
   {
      return new QuadrupedSupportPlanarRegionParametersMessagePubSubType();
   }
}
