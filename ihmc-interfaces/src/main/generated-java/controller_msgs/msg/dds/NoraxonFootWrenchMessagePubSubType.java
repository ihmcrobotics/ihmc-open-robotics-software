package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "NoraxonFootWrenchMessage" defined in "NoraxonFootWrenchMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from NoraxonFootWrenchMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit NoraxonFootWrenchMessage_.idl instead.
*
*/
public class NoraxonFootWrenchMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.NoraxonFootWrenchMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::NoraxonFootWrenchMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.NoraxonFootWrenchMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.NoraxonFootWrenchMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.NoraxonFootWrenchMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.NoraxonFootWrenchMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.NoraxonFootWrenchMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getRobotSide());

      cdr.write_type_6(data.getLinearForceX());

      cdr.write_type_6(data.getLinearForceY());

      cdr.write_type_6(data.getLinearForceZ());

      cdr.write_type_6(data.getAngularMomentX());

      cdr.write_type_6(data.getAngularMomentY());

      cdr.write_type_6(data.getAngularMomentZ());

   }

   public static void read(controller_msgs.msg.dds.NoraxonFootWrenchMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRobotSide(cdr.read_type_9());
      	
      data.setLinearForceX(cdr.read_type_6());
      	
      data.setLinearForceY(cdr.read_type_6());
      	
      data.setLinearForceZ(cdr.read_type_6());
      	
      data.setAngularMomentX(cdr.read_type_6());
      	
      data.setAngularMomentY(cdr.read_type_6());
      	
      data.setAngularMomentZ(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.NoraxonFootWrenchMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_6("linear_force_x", data.getLinearForceX());
      ser.write_type_6("linear_force_y", data.getLinearForceY());
      ser.write_type_6("linear_force_z", data.getLinearForceZ());
      ser.write_type_6("angular_moment_x", data.getAngularMomentX());
      ser.write_type_6("angular_moment_y", data.getAngularMomentY());
      ser.write_type_6("angular_moment_z", data.getAngularMomentZ());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.NoraxonFootWrenchMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRobotSide(ser.read_type_9("robot_side"));
      data.setLinearForceX(ser.read_type_6("linear_force_x"));
      data.setLinearForceY(ser.read_type_6("linear_force_y"));
      data.setLinearForceZ(ser.read_type_6("linear_force_z"));
      data.setAngularMomentX(ser.read_type_6("angular_moment_x"));
      data.setAngularMomentY(ser.read_type_6("angular_moment_y"));
      data.setAngularMomentZ(ser.read_type_6("angular_moment_z"));
   }

   public static void staticCopy(controller_msgs.msg.dds.NoraxonFootWrenchMessage src, controller_msgs.msg.dds.NoraxonFootWrenchMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.NoraxonFootWrenchMessage createData()
   {
      return new controller_msgs.msg.dds.NoraxonFootWrenchMessage();
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
   
   public void serialize(controller_msgs.msg.dds.NoraxonFootWrenchMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.NoraxonFootWrenchMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.NoraxonFootWrenchMessage src, controller_msgs.msg.dds.NoraxonFootWrenchMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public NoraxonFootWrenchMessagePubSubType newInstance()
   {
      return new NoraxonFootWrenchMessagePubSubType();
   }
}
