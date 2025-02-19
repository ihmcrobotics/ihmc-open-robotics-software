package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ObjectCarryMessage" defined in "ObjectCarryMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ObjectCarryMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ObjectCarryMessage_.idl instead.
*
*/
public class ObjectCarryMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ObjectCarryMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ObjectCarryMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "d8669ffb739edbd8055c6f1132b610dd957c4bbbd44a8bf080187514ad1877b2";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ObjectCarryMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ObjectCarryMessage data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ObjectCarryMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ObjectCarryMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getObjectPose(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ObjectCarryMessage data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.PosePubSubType.write(data.getObjectPose(), cdr);
      cdr.write_type_7(data.getObjectIsCarried());

   }

   public static void read(controller_msgs.msg.dds.ObjectCarryMessage data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.PosePubSubType.read(data.getObjectPose(), cdr);	
      data.setObjectIsCarried(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ObjectCarryMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("object_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getObjectPose());

      ser.write_type_7("object_is_carried", data.getObjectIsCarried());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ObjectCarryMessage data)
   {
      ser.read_type_a("object_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getObjectPose());

      data.setObjectIsCarried(ser.read_type_7("object_is_carried"));
   }

   public static void staticCopy(controller_msgs.msg.dds.ObjectCarryMessage src, controller_msgs.msg.dds.ObjectCarryMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ObjectCarryMessage createData()
   {
      return new controller_msgs.msg.dds.ObjectCarryMessage();
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
   
   public void serialize(controller_msgs.msg.dds.ObjectCarryMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ObjectCarryMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ObjectCarryMessage src, controller_msgs.msg.dds.ObjectCarryMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ObjectCarryMessagePubSubType newInstance()
   {
      return new ObjectCarryMessagePubSubType();
   }
}
