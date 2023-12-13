package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BipedalSupportPlanarRegionParametersMessage" defined in "BipedalSupportPlanarRegionParametersMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BipedalSupportPlanarRegionParametersMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BipedalSupportPlanarRegionParametersMessage_.idl instead.
*
*/
public class BipedalSupportPlanarRegionParametersMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.BipedalSupportPlanarRegionParametersMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::BipedalSupportPlanarRegionParametersMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "a2d046662cdc793ae3dfc4dcdd63b269bd5213e5cbc292d2f5361c8d5aa4e690";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.BipedalSupportPlanarRegionParametersMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.BipedalSupportPlanarRegionParametersMessage data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.BipedalSupportPlanarRegionParametersMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.BipedalSupportPlanarRegionParametersMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.BipedalSupportPlanarRegionParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_7(data.getEnable());

      cdr.write_type_6(data.getSupportRegionScaleFactor());

   }

   public static void read(controller_msgs.msg.dds.BipedalSupportPlanarRegionParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setEnable(cdr.read_type_7());
      	
      data.setSupportRegionScaleFactor(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.BipedalSupportPlanarRegionParametersMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("enable", data.getEnable());
      ser.write_type_6("support_region_scale_factor", data.getSupportRegionScaleFactor());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.BipedalSupportPlanarRegionParametersMessage data)
   {
      data.setEnable(ser.read_type_7("enable"));
      data.setSupportRegionScaleFactor(ser.read_type_6("support_region_scale_factor"));
   }

   public static void staticCopy(controller_msgs.msg.dds.BipedalSupportPlanarRegionParametersMessage src, controller_msgs.msg.dds.BipedalSupportPlanarRegionParametersMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.BipedalSupportPlanarRegionParametersMessage createData()
   {
      return new controller_msgs.msg.dds.BipedalSupportPlanarRegionParametersMessage();
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
   
   public void serialize(controller_msgs.msg.dds.BipedalSupportPlanarRegionParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.BipedalSupportPlanarRegionParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.BipedalSupportPlanarRegionParametersMessage src, controller_msgs.msg.dds.BipedalSupportPlanarRegionParametersMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BipedalSupportPlanarRegionParametersMessagePubSubType newInstance()
   {
      return new BipedalSupportPlanarRegionParametersMessagePubSubType();
   }
}
