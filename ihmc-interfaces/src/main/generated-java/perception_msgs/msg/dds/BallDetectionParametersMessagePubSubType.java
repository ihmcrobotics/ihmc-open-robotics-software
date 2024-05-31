package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BallDetectionParametersMessage" defined in "BallDetectionParametersMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BallDetectionParametersMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BallDetectionParametersMessage_.idl instead.
*
*/
public class BallDetectionParametersMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.BallDetectionParametersMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::BallDetectionParametersMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "a5b901b96ec1ff4f9c34c142abe92c764b8a306af6f7bccf0d74d3c53f719357";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.BallDetectionParametersMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.BallDetectionParametersMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.BallDetectionParametersMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.BallDetectionParametersMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.BallDetectionParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_6(data.getBallDiameter());

      cdr.write_type_6(data.getAlpha());

      cdr.write_type_6(data.getHLow());

      cdr.write_type_6(data.getSLow());

      cdr.write_type_6(data.getVLow());

      cdr.write_type_6(data.getHHigh());

      cdr.write_type_6(data.getSHigh());

      cdr.write_type_6(data.getVHigh());

   }

   public static void read(perception_msgs.msg.dds.BallDetectionParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setBallDiameter(cdr.read_type_6());
      	
      data.setAlpha(cdr.read_type_6());
      	
      data.setHLow(cdr.read_type_6());
      	
      data.setSLow(cdr.read_type_6());
      	
      data.setVLow(cdr.read_type_6());
      	
      data.setHHigh(cdr.read_type_6());
      	
      data.setSHigh(cdr.read_type_6());
      	
      data.setVHigh(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.BallDetectionParametersMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_6("ball_diameter", data.getBallDiameter());
      ser.write_type_6("alpha", data.getAlpha());
      ser.write_type_6("h_low", data.getHLow());
      ser.write_type_6("s_low", data.getSLow());
      ser.write_type_6("v_low", data.getVLow());
      ser.write_type_6("h_high", data.getHHigh());
      ser.write_type_6("s_high", data.getSHigh());
      ser.write_type_6("v_high", data.getVHigh());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.BallDetectionParametersMessage data)
   {
      data.setBallDiameter(ser.read_type_6("ball_diameter"));
      data.setAlpha(ser.read_type_6("alpha"));
      data.setHLow(ser.read_type_6("h_low"));
      data.setSLow(ser.read_type_6("s_low"));
      data.setVLow(ser.read_type_6("v_low"));
      data.setHHigh(ser.read_type_6("h_high"));
      data.setSHigh(ser.read_type_6("s_high"));
      data.setVHigh(ser.read_type_6("v_high"));
   }

   public static void staticCopy(perception_msgs.msg.dds.BallDetectionParametersMessage src, perception_msgs.msg.dds.BallDetectionParametersMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.BallDetectionParametersMessage createData()
   {
      return new perception_msgs.msg.dds.BallDetectionParametersMessage();
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
   
   public void serialize(perception_msgs.msg.dds.BallDetectionParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.BallDetectionParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.BallDetectionParametersMessage src, perception_msgs.msg.dds.BallDetectionParametersMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BallDetectionParametersMessagePubSubType newInstance()
   {
      return new BallDetectionParametersMessagePubSubType();
   }
}
