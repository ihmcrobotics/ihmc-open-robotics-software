package quadruped_msgs.msg.dds;

/**
* 
* Topic data type of the struct "QuadrupedSupportPlanarRegionParametersMessage" defined in "QuadrupedSupportPlanarRegionParametersMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from QuadrupedSupportPlanarRegionParametersMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit QuadrupedSupportPlanarRegionParametersMessage_.idl instead.
*
*/
public class QuadrupedSupportPlanarRegionParametersMessagePubSubType implements us.ihmc.pubsub.TopicDataType<quadruped_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage>
{
   public static final java.lang.String name = "quadruped_msgs::msg::dds_::QuadrupedSupportPlanarRegionParametersMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "a03f44ed44faa2f5f5a869b8604abc43908eebca980e70a0e4697739340c4e43";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(quadruped_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, quadruped_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(quadruped_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(quadruped_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(quadruped_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_7(data.getEnable());

      cdr.write_type_6(data.getInsideSupportRegionSize());

      cdr.write_type_6(data.getOutsideSupportRegionSize());

   }

   public static void read(quadruped_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setEnable(cdr.read_type_7());
      	
      data.setInsideSupportRegionSize(cdr.read_type_6());
      	
      data.setOutsideSupportRegionSize(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(quadruped_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("enable", data.getEnable());
      ser.write_type_6("inside_support_region_size", data.getInsideSupportRegionSize());
      ser.write_type_6("outside_support_region_size", data.getOutsideSupportRegionSize());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, quadruped_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage data)
   {
      data.setEnable(ser.read_type_7("enable"));
      data.setInsideSupportRegionSize(ser.read_type_6("inside_support_region_size"));
      data.setOutsideSupportRegionSize(ser.read_type_6("outside_support_region_size"));
   }

   public static void staticCopy(quadruped_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage src, quadruped_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage dest)
   {
      dest.set(src);
   }

   @Override
   public quadruped_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage createData()
   {
      return new quadruped_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage();
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
   
   public void serialize(quadruped_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(quadruped_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(quadruped_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage src, quadruped_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public QuadrupedSupportPlanarRegionParametersMessagePubSubType newInstance()
   {
      return new QuadrupedSupportPlanarRegionParametersMessagePubSubType();
   }
}
