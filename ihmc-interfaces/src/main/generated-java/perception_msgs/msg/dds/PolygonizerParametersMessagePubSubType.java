package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PolygonizerParametersMessage" defined in "PolygonizerParametersMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PolygonizerParametersMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PolygonizerParametersMessage_.idl instead.
*
*/
public class PolygonizerParametersMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.PolygonizerParametersMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::PolygonizerParametersMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "5796ab7dac2bc9ebc03ec7251a70a35027ffae3d06e329407472b3c68141f7a6";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.PolygonizerParametersMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.PolygonizerParametersMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.PolygonizerParametersMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.PolygonizerParametersMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.PolygonizerParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_6(data.getConcaveHullThreshold());

      cdr.write_type_2(data.getMinNumberOfNodes());

      cdr.write_type_6(data.getShallowAngleThreshold());

      cdr.write_type_6(data.getPeakAngleThreshold());

      cdr.write_type_6(data.getLengthThreshold());

      cdr.write_type_6(data.getDepthThreshold());

      cdr.write_type_7(data.getCutNarrowPassage());

   }

   public static void read(perception_msgs.msg.dds.PolygonizerParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setConcaveHullThreshold(cdr.read_type_6());
      	
      data.setMinNumberOfNodes(cdr.read_type_2());
      	
      data.setShallowAngleThreshold(cdr.read_type_6());
      	
      data.setPeakAngleThreshold(cdr.read_type_6());
      	
      data.setLengthThreshold(cdr.read_type_6());
      	
      data.setDepthThreshold(cdr.read_type_6());
      	
      data.setCutNarrowPassage(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.PolygonizerParametersMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_6("concave_hull_threshold", data.getConcaveHullThreshold());
      ser.write_type_2("min_number_of_nodes", data.getMinNumberOfNodes());
      ser.write_type_6("shallow_angle_threshold", data.getShallowAngleThreshold());
      ser.write_type_6("peak_angle_threshold", data.getPeakAngleThreshold());
      ser.write_type_6("length_threshold", data.getLengthThreshold());
      ser.write_type_6("depth_threshold", data.getDepthThreshold());
      ser.write_type_7("cut_narrow_passage", data.getCutNarrowPassage());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.PolygonizerParametersMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setConcaveHullThreshold(ser.read_type_6("concave_hull_threshold"));
      data.setMinNumberOfNodes(ser.read_type_2("min_number_of_nodes"));
      data.setShallowAngleThreshold(ser.read_type_6("shallow_angle_threshold"));
      data.setPeakAngleThreshold(ser.read_type_6("peak_angle_threshold"));
      data.setLengthThreshold(ser.read_type_6("length_threshold"));
      data.setDepthThreshold(ser.read_type_6("depth_threshold"));
      data.setCutNarrowPassage(ser.read_type_7("cut_narrow_passage"));
   }

   public static void staticCopy(perception_msgs.msg.dds.PolygonizerParametersMessage src, perception_msgs.msg.dds.PolygonizerParametersMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.PolygonizerParametersMessage createData()
   {
      return new perception_msgs.msg.dds.PolygonizerParametersMessage();
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
   
   public void serialize(perception_msgs.msg.dds.PolygonizerParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.PolygonizerParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.PolygonizerParametersMessage src, perception_msgs.msg.dds.PolygonizerParametersMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PolygonizerParametersMessagePubSubType newInstance()
   {
      return new PolygonizerParametersMessagePubSubType();
   }
}
