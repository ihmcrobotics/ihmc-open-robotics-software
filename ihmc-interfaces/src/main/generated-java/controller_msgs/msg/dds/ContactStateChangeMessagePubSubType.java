package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ContactStateChangeMessage" defined in "ContactStateChangeMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ContactStateChangeMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ContactStateChangeMessage_.idl instead.
*
*/
public class ContactStateChangeMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ContactStateChangeMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ContactStateChangeMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ContactStateChangeMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ContactStateChangeMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ContactStateChangeMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ContactStateChangeMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ContactStateChangeMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_6(data.getTime());

      cdr.write_type_2(data.getRigidBodyHashCode());

      cdr.write_type_7(data.getAddContactPoint());

   }

   public static void read(controller_msgs.msg.dds.ContactStateChangeMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setTime(cdr.read_type_6());
      	
      data.setRigidBodyHashCode(cdr.read_type_2());
      	
      data.setAddContactPoint(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ContactStateChangeMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_6("time", data.getTime());
      ser.write_type_2("rigid_body_hash_code", data.getRigidBodyHashCode());
      ser.write_type_7("add_contact_point", data.getAddContactPoint());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ContactStateChangeMessage data)
   {
      data.setTime(ser.read_type_6("time"));
      data.setRigidBodyHashCode(ser.read_type_2("rigid_body_hash_code"));
      data.setAddContactPoint(ser.read_type_7("add_contact_point"));
   }

   public static void staticCopy(controller_msgs.msg.dds.ContactStateChangeMessage src, controller_msgs.msg.dds.ContactStateChangeMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ContactStateChangeMessage createData()
   {
      return new controller_msgs.msg.dds.ContactStateChangeMessage();
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
   
   public void serialize(controller_msgs.msg.dds.ContactStateChangeMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ContactStateChangeMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ContactStateChangeMessage src, controller_msgs.msg.dds.ContactStateChangeMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ContactStateChangeMessagePubSubType newInstance()
   {
      return new ContactStateChangeMessagePubSubType();
   }
}
