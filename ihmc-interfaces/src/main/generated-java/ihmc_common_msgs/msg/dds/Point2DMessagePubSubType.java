package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "Point2DMessage" defined in "Point2DMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Point2DMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Point2DMessage_.idl instead.
*
*/
public class Point2DMessagePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.Point2DMessage>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::Point2DMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "84a58716726e3af372362df1cb04897644241284ab466ce1fcd4e2ade344083e";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.Point2DMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.Point2DMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.Point2DMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.Point2DMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.Point2DMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_6(data.getX());

      cdr.write_type_6(data.getY());

   }

   public static void read(ihmc_common_msgs.msg.dds.Point2DMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setX(cdr.read_type_6());
      	
      data.setY(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.Point2DMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_6("x", data.getX());
      ser.write_type_6("y", data.getY());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.Point2DMessage data)
   {
      data.setX(ser.read_type_6("x"));
      data.setY(ser.read_type_6("y"));
   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.Point2DMessage src, ihmc_common_msgs.msg.dds.Point2DMessage dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.Point2DMessage createData()
   {
      return new ihmc_common_msgs.msg.dds.Point2DMessage();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.Point2DMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.Point2DMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.Point2DMessage src, ihmc_common_msgs.msg.dds.Point2DMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public Point2DMessagePubSubType newInstance()
   {
      return new Point2DMessagePubSubType();
   }
}
