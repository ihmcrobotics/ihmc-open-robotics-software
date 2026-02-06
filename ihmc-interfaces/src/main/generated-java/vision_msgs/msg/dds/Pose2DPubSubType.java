package vision_msgs.msg.dds;

/**
* 
* Topic data type of the struct "Pose2D" defined in "Pose2D_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Pose2D_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Pose2D_.idl instead.
*
*/
public class Pose2DPubSubType implements us.ihmc.pubsub.TopicDataType<vision_msgs.msg.dds.Pose2D>
{
   public static final java.lang.String name = "vision_msgs::msg::dds_::Pose2D_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "c6aa91a138941585fb214cf909381f5aa586d268bdd78c64741ecbd497512257";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(vision_msgs.msg.dds.Pose2D data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, vision_msgs.msg.dds.Pose2D data) throws java.io.IOException
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

      current_alignment += vision_msgs.msg.dds.Point2DPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(vision_msgs.msg.dds.Pose2D data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(vision_msgs.msg.dds.Pose2D data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += vision_msgs.msg.dds.Point2DPubSubType.getCdrSerializedSize(data.getPosition(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(vision_msgs.msg.dds.Pose2D data, us.ihmc.idl.CDR cdr)
   {
      vision_msgs.msg.dds.Point2DPubSubType.write(data.getPosition(), cdr);
      cdr.write_type_6(data.getTheta());

   }

   public static void read(vision_msgs.msg.dds.Pose2D data, us.ihmc.idl.CDR cdr)
   {
      vision_msgs.msg.dds.Point2DPubSubType.read(data.getPosition(), cdr);	
      data.setTheta(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(vision_msgs.msg.dds.Pose2D data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("position", new vision_msgs.msg.dds.Point2DPubSubType(), data.getPosition());

      ser.write_type_6("theta", data.getTheta());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, vision_msgs.msg.dds.Pose2D data)
   {
      ser.read_type_a("position", new vision_msgs.msg.dds.Point2DPubSubType(), data.getPosition());

      data.setTheta(ser.read_type_6("theta"));
   }

   public static void staticCopy(vision_msgs.msg.dds.Pose2D src, vision_msgs.msg.dds.Pose2D dest)
   {
      dest.set(src);
   }

   @Override
   public vision_msgs.msg.dds.Pose2D createData()
   {
      return new vision_msgs.msg.dds.Pose2D();
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
   
   public void serialize(vision_msgs.msg.dds.Pose2D data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(vision_msgs.msg.dds.Pose2D data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(vision_msgs.msg.dds.Pose2D src, vision_msgs.msg.dds.Pose2D dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public Pose2DPubSubType newInstance()
   {
      return new Pose2DPubSubType();
   }
}
