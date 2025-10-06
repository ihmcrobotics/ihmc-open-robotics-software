package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "KinematicsToolboxFootStatus" defined in "KinematicsToolboxFootStatus_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from KinematicsToolboxFootStatus_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit KinematicsToolboxFootStatus_.idl instead.
*
*/
public class KinematicsToolboxFootStatusPubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.KinematicsToolboxFootStatus>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::KinematicsToolboxFootStatus_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "8aea7f2e2700e6870d70e177ddd37ec4cc0b20188a6da5a8c21f453d6651df86";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.KinematicsToolboxFootStatus data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.KinematicsToolboxFootStatus data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.KinematicsToolboxFootStatus data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.KinematicsToolboxFootStatus data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.KinematicsToolboxFootStatus data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getSide());

      cdr.write_type_7(data.getFootInContact());

   }

   public static void read(toolbox_msgs.msg.dds.KinematicsToolboxFootStatus data, us.ihmc.idl.CDR cdr)
   {
      data.setSide(cdr.read_type_9());
      	
      data.setFootInContact(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.KinematicsToolboxFootStatus data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("side", data.getSide());
      ser.write_type_7("foot_in_contact", data.getFootInContact());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.KinematicsToolboxFootStatus data)
   {
      data.setSide(ser.read_type_9("side"));
      data.setFootInContact(ser.read_type_7("foot_in_contact"));
   }

   public static void staticCopy(toolbox_msgs.msg.dds.KinematicsToolboxFootStatus src, toolbox_msgs.msg.dds.KinematicsToolboxFootStatus dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.KinematicsToolboxFootStatus createData()
   {
      return new toolbox_msgs.msg.dds.KinematicsToolboxFootStatus();
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
   
   public void serialize(toolbox_msgs.msg.dds.KinematicsToolboxFootStatus data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.KinematicsToolboxFootStatus data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.KinematicsToolboxFootStatus src, toolbox_msgs.msg.dds.KinematicsToolboxFootStatus dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public KinematicsToolboxFootStatusPubSubType newInstance()
   {
      return new KinematicsToolboxFootStatusPubSubType();
   }
}
