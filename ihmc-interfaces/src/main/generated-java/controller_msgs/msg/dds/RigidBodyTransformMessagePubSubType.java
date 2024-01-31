package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "RigidBodyTransformMessage" defined in "RigidBodyTransformMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from RigidBodyTransformMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit RigidBodyTransformMessage_.idl instead.
*
*/
public class RigidBodyTransformMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.RigidBodyTransformMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::RigidBodyTransformMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "748bf6f331ed9d3d3b8da0edf07d91422f9ff59111d75aa8c3ab15e2c43aec92";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.RigidBodyTransformMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.RigidBodyTransformMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.RigidBodyTransformMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.RigidBodyTransformMessage data, int current_alignment)
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


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.RigidBodyTransformMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_6(data.getX());

      cdr.write_type_6(data.getY());

      cdr.write_type_6(data.getZ());

      cdr.write_type_6(data.getM00());

      cdr.write_type_6(data.getM01());

      cdr.write_type_6(data.getM02());

      cdr.write_type_6(data.getM10());

      cdr.write_type_6(data.getM11());

      cdr.write_type_6(data.getM12());

      cdr.write_type_6(data.getM20());

      cdr.write_type_6(data.getM21());

      cdr.write_type_6(data.getM22());

   }

   public static void read(controller_msgs.msg.dds.RigidBodyTransformMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setX(cdr.read_type_6());
      	
      data.setY(cdr.read_type_6());
      	
      data.setZ(cdr.read_type_6());
      	
      data.setM00(cdr.read_type_6());
      	
      data.setM01(cdr.read_type_6());
      	
      data.setM02(cdr.read_type_6());
      	
      data.setM10(cdr.read_type_6());
      	
      data.setM11(cdr.read_type_6());
      	
      data.setM12(cdr.read_type_6());
      	
      data.setM20(cdr.read_type_6());
      	
      data.setM21(cdr.read_type_6());
      	
      data.setM22(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.RigidBodyTransformMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_6("x", data.getX());
      ser.write_type_6("y", data.getY());
      ser.write_type_6("z", data.getZ());
      ser.write_type_6("m00", data.getM00());
      ser.write_type_6("m01", data.getM01());
      ser.write_type_6("m02", data.getM02());
      ser.write_type_6("m10", data.getM10());
      ser.write_type_6("m11", data.getM11());
      ser.write_type_6("m12", data.getM12());
      ser.write_type_6("m20", data.getM20());
      ser.write_type_6("m21", data.getM21());
      ser.write_type_6("m22", data.getM22());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.RigidBodyTransformMessage data)
   {
      data.setX(ser.read_type_6("x"));
      data.setY(ser.read_type_6("y"));
      data.setZ(ser.read_type_6("z"));
      data.setM00(ser.read_type_6("m00"));
      data.setM01(ser.read_type_6("m01"));
      data.setM02(ser.read_type_6("m02"));
      data.setM10(ser.read_type_6("m10"));
      data.setM11(ser.read_type_6("m11"));
      data.setM12(ser.read_type_6("m12"));
      data.setM20(ser.read_type_6("m20"));
      data.setM21(ser.read_type_6("m21"));
      data.setM22(ser.read_type_6("m22"));
   }

   public static void staticCopy(controller_msgs.msg.dds.RigidBodyTransformMessage src, controller_msgs.msg.dds.RigidBodyTransformMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.RigidBodyTransformMessage createData()
   {
      return new controller_msgs.msg.dds.RigidBodyTransformMessage();
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
   
   public void serialize(controller_msgs.msg.dds.RigidBodyTransformMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.RigidBodyTransformMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.RigidBodyTransformMessage src, controller_msgs.msg.dds.RigidBodyTransformMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public RigidBodyTransformMessagePubSubType newInstance()
   {
      return new RigidBodyTransformMessagePubSubType();
   }
}
