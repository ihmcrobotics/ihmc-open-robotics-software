package unitree_go_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PathPoint" defined in "PathPoint_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PathPoint_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PathPoint_.idl instead.
*
*/
public class PathPointPubSubType implements us.ihmc.pubsub.TopicDataType<unitree_go_msgs.msg.dds.PathPoint>
{
   public static final java.lang.String name = "unitree_go_msgs::msg::dds_::PathPoint_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "aaaaaadcebfe29e999a98d95a0ffc2921906b44bb7378e1b1ec57d3eec22e3f0";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(unitree_go_msgs.msg.dds.PathPoint data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, unitree_go_msgs.msg.dds.PathPoint data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.PathPoint data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.PathPoint data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(unitree_go_msgs.msg.dds.PathPoint data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_5(data.getTFromStart());

      cdr.write_type_5(data.getX());

      cdr.write_type_5(data.getY());

      cdr.write_type_5(data.getYaw());

      cdr.write_type_5(data.getVx());

      cdr.write_type_5(data.getVy());

      cdr.write_type_5(data.getVyaw());

   }

   public static void read(unitree_go_msgs.msg.dds.PathPoint data, us.ihmc.idl.CDR cdr)
   {
      data.setTFromStart(cdr.read_type_5());
      	
      data.setX(cdr.read_type_5());
      	
      data.setY(cdr.read_type_5());
      	
      data.setYaw(cdr.read_type_5());
      	
      data.setVx(cdr.read_type_5());
      	
      data.setVy(cdr.read_type_5());
      	
      data.setVyaw(cdr.read_type_5());
      	

   }

   @Override
   public final void serialize(unitree_go_msgs.msg.dds.PathPoint data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_5("t_from_start", data.getTFromStart());
      ser.write_type_5("x", data.getX());
      ser.write_type_5("y", data.getY());
      ser.write_type_5("yaw", data.getYaw());
      ser.write_type_5("vx", data.getVx());
      ser.write_type_5("vy", data.getVy());
      ser.write_type_5("vyaw", data.getVyaw());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, unitree_go_msgs.msg.dds.PathPoint data)
   {
      data.setTFromStart(ser.read_type_5("t_from_start"));
      data.setX(ser.read_type_5("x"));
      data.setY(ser.read_type_5("y"));
      data.setYaw(ser.read_type_5("yaw"));
      data.setVx(ser.read_type_5("vx"));
      data.setVy(ser.read_type_5("vy"));
      data.setVyaw(ser.read_type_5("vyaw"));
   }

   public static void staticCopy(unitree_go_msgs.msg.dds.PathPoint src, unitree_go_msgs.msg.dds.PathPoint dest)
   {
      dest.set(src);
   }

   @Override
   public unitree_go_msgs.msg.dds.PathPoint createData()
   {
      return new unitree_go_msgs.msg.dds.PathPoint();
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
   
   public void serialize(unitree_go_msgs.msg.dds.PathPoint data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(unitree_go_msgs.msg.dds.PathPoint data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(unitree_go_msgs.msg.dds.PathPoint src, unitree_go_msgs.msg.dds.PathPoint dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PathPointPubSubType newInstance()
   {
      return new PathPointPubSubType();
   }
}
