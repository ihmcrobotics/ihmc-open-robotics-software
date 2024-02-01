package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ConcaveHullFactoryParametersMessage" defined in "ConcaveHullFactoryParametersMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ConcaveHullFactoryParametersMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ConcaveHullFactoryParametersMessage_.idl instead.
*
*/
public class ConcaveHullFactoryParametersMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ConcaveHullFactoryParametersMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ConcaveHullFactoryParametersMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "09f050c5ac4416d7bf6246005c4e9679d125a9a38ae63766bc213c7ec169d5bc";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ConcaveHullFactoryParametersMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ConcaveHullFactoryParametersMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ConcaveHullFactoryParametersMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ConcaveHullFactoryParametersMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ConcaveHullFactoryParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_6(data.getEdgeLengthThreshold());

      cdr.write_type_7(data.getRemoveAllTrianglesWithTwoBorderEdges());

      cdr.write_type_7(data.getAllowSplittingConcaveHull());

      cdr.write_type_2(data.getMaxNumberOfIterations());

      cdr.write_type_6(data.getTriangulationTolerance());

   }

   public static void read(controller_msgs.msg.dds.ConcaveHullFactoryParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setEdgeLengthThreshold(cdr.read_type_6());
      	
      data.setRemoveAllTrianglesWithTwoBorderEdges(cdr.read_type_7());
      	
      data.setAllowSplittingConcaveHull(cdr.read_type_7());
      	
      data.setMaxNumberOfIterations(cdr.read_type_2());
      	
      data.setTriangulationTolerance(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ConcaveHullFactoryParametersMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_6("edge_length_threshold", data.getEdgeLengthThreshold());
      ser.write_type_7("remove_all_triangles_with_two_border_edges", data.getRemoveAllTrianglesWithTwoBorderEdges());
      ser.write_type_7("allow_splitting_concave_hull", data.getAllowSplittingConcaveHull());
      ser.write_type_2("max_number_of_iterations", data.getMaxNumberOfIterations());
      ser.write_type_6("triangulation_tolerance", data.getTriangulationTolerance());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ConcaveHullFactoryParametersMessage data)
   {
      data.setEdgeLengthThreshold(ser.read_type_6("edge_length_threshold"));
      data.setRemoveAllTrianglesWithTwoBorderEdges(ser.read_type_7("remove_all_triangles_with_two_border_edges"));
      data.setAllowSplittingConcaveHull(ser.read_type_7("allow_splitting_concave_hull"));
      data.setMaxNumberOfIterations(ser.read_type_2("max_number_of_iterations"));
      data.setTriangulationTolerance(ser.read_type_6("triangulation_tolerance"));
   }

   public static void staticCopy(controller_msgs.msg.dds.ConcaveHullFactoryParametersMessage src, controller_msgs.msg.dds.ConcaveHullFactoryParametersMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ConcaveHullFactoryParametersMessage createData()
   {
      return new controller_msgs.msg.dds.ConcaveHullFactoryParametersMessage();
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
   
   public void serialize(controller_msgs.msg.dds.ConcaveHullFactoryParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ConcaveHullFactoryParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ConcaveHullFactoryParametersMessage src, controller_msgs.msg.dds.ConcaveHullFactoryParametersMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ConcaveHullFactoryParametersMessagePubSubType newInstance()
   {
      return new ConcaveHullFactoryParametersMessagePubSubType();
   }
}
