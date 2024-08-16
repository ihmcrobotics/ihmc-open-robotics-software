package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BimanualManipulationMessage" defined in "BimanualManipulationMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BimanualManipulationMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BimanualManipulationMessage_.idl instead.
*
*/
public class BimanualManipulationMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.BimanualManipulationMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::BimanualManipulationMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "6c7541efe842bd33a63c340a833ad89de7e5778809d2d739c742c0c746d32846";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.BimanualManipulationMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.BimanualManipulationMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.BimanualManipulationMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.BimanualManipulationMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.BimanualManipulationMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_7(data.getDisable());

      cdr.write_type_6(data.getObjectMass());

      cdr.write_type_6(data.getSqueezeForce());

      cdr.write_type_6(data.getInitializeDuration());

      cdr.write_type_6(data.getAcceptableTrackingError());

   }

   public static void read(controller_msgs.msg.dds.BimanualManipulationMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setDisable(cdr.read_type_7());
      	
      data.setObjectMass(cdr.read_type_6());
      	
      data.setSqueezeForce(cdr.read_type_6());
      	
      data.setInitializeDuration(cdr.read_type_6());
      	
      data.setAcceptableTrackingError(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.BimanualManipulationMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("disable", data.getDisable());
      ser.write_type_6("object_mass", data.getObjectMass());
      ser.write_type_6("squeeze_force", data.getSqueezeForce());
      ser.write_type_6("initialize_duration", data.getInitializeDuration());
      ser.write_type_6("acceptable_tracking_error", data.getAcceptableTrackingError());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.BimanualManipulationMessage data)
   {
      data.setDisable(ser.read_type_7("disable"));
      data.setObjectMass(ser.read_type_6("object_mass"));
      data.setSqueezeForce(ser.read_type_6("squeeze_force"));
      data.setInitializeDuration(ser.read_type_6("initialize_duration"));
      data.setAcceptableTrackingError(ser.read_type_6("acceptable_tracking_error"));
   }

   public static void staticCopy(controller_msgs.msg.dds.BimanualManipulationMessage src, controller_msgs.msg.dds.BimanualManipulationMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.BimanualManipulationMessage createData()
   {
      return new controller_msgs.msg.dds.BimanualManipulationMessage();
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
   
   public void serialize(controller_msgs.msg.dds.BimanualManipulationMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.BimanualManipulationMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.BimanualManipulationMessage src, controller_msgs.msg.dds.BimanualManipulationMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BimanualManipulationMessagePubSubType newInstance()
   {
      return new BimanualManipulationMessagePubSubType();
   }
}
