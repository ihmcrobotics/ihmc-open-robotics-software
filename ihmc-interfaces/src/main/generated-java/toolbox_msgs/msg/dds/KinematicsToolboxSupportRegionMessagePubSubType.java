package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "KinematicsToolboxSupportRegionMessage" defined in "KinematicsToolboxSupportRegionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from KinematicsToolboxSupportRegionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit KinematicsToolboxSupportRegionMessage_.idl instead.
*
*/
public class KinematicsToolboxSupportRegionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::KinematicsToolboxSupportRegionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "871c5f0dfd09c0a4e52410ab3ce8cb63589b5a35e265a68bd1d418d4c0cf233b";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getSupportRegionVertices().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getSupportRegionVertices().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getSupportRegionVertexFrames().size() * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_6(data.getCenterOfMassMargin());

      if(data.getSupportRegionVertices().size() <= 100)
      cdr.write_type_e(data.getSupportRegionVertices());else
          throw new RuntimeException("support_region_vertices field exceeds the maximum length");

      if(data.getSupportRegionVertexFrames().size() <= 100)
      cdr.write_type_e(data.getSupportRegionVertexFrames());else
          throw new RuntimeException("support_region_vertex_frames field exceeds the maximum length");

   }

   public static void read(toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setCenterOfMassMargin(cdr.read_type_6());
      	
      cdr.read_type_e(data.getSupportRegionVertices());	
      cdr.read_type_e(data.getSupportRegionVertexFrames());	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_6("center_of_mass_margin", data.getCenterOfMassMargin());
      ser.write_type_e("support_region_vertices", data.getSupportRegionVertices());
      ser.write_type_e("support_region_vertex_frames", data.getSupportRegionVertexFrames());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setCenterOfMassMargin(ser.read_type_6("center_of_mass_margin"));
      ser.read_type_e("support_region_vertices", data.getSupportRegionVertices());
      ser.read_type_e("support_region_vertex_frames", data.getSupportRegionVertexFrames());
   }

   public static void staticCopy(toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionMessage src, toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionMessage createData()
   {
      return new toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionMessage src, toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public KinematicsToolboxSupportRegionMessagePubSubType newInstance()
   {
      return new KinematicsToolboxSupportRegionMessagePubSubType();
   }
}
