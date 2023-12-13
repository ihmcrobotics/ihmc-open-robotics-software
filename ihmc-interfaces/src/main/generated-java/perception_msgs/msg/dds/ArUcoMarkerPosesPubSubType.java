package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ArUcoMarkerPoses" defined in "ArUcoMarkerPoses_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ArUcoMarkerPoses_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ArUcoMarkerPoses_.idl instead.
*
*/
public class ArUcoMarkerPosesPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.ArUcoMarkerPoses>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::ArUcoMarkerPoses_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "3f715fd2a14a20adfc990af912b8918e395d4327fa70447abe4ccbcc7ff713de";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.ArUcoMarkerPoses data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.ArUcoMarkerPoses data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.ArUcoMarkerPoses data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.ArUcoMarkerPoses data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getMarkerId().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getPosition().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getPosition().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getOrientation().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getOrientation().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.ArUcoMarkerPoses data, us.ihmc.idl.CDR cdr)
   {
      if(data.getMarkerId().size() <= 100)
      cdr.write_type_e(data.getMarkerId());else
          throw new RuntimeException("marker_id field exceeds the maximum length");

      if(data.getPosition().size() <= 100)
      cdr.write_type_e(data.getPosition());else
          throw new RuntimeException("position field exceeds the maximum length");

      if(data.getOrientation().size() <= 100)
      cdr.write_type_e(data.getOrientation());else
          throw new RuntimeException("orientation field exceeds the maximum length");

   }

   public static void read(perception_msgs.msg.dds.ArUcoMarkerPoses data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getMarkerId());	
      cdr.read_type_e(data.getPosition());	
      cdr.read_type_e(data.getOrientation());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.ArUcoMarkerPoses data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("marker_id", data.getMarkerId());
      ser.write_type_e("position", data.getPosition());
      ser.write_type_e("orientation", data.getOrientation());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.ArUcoMarkerPoses data)
   {
      ser.read_type_e("marker_id", data.getMarkerId());
      ser.read_type_e("position", data.getPosition());
      ser.read_type_e("orientation", data.getOrientation());
   }

   public static void staticCopy(perception_msgs.msg.dds.ArUcoMarkerPoses src, perception_msgs.msg.dds.ArUcoMarkerPoses dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.ArUcoMarkerPoses createData()
   {
      return new perception_msgs.msg.dds.ArUcoMarkerPoses();
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
   
   public void serialize(perception_msgs.msg.dds.ArUcoMarkerPoses data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.ArUcoMarkerPoses data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.ArUcoMarkerPoses src, perception_msgs.msg.dds.ArUcoMarkerPoses dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ArUcoMarkerPosesPubSubType newInstance()
   {
      return new ArUcoMarkerPosesPubSubType();
   }
}
