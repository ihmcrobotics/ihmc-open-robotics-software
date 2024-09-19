package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "KinematicsToolboxSupportRegionDebug" defined in "KinematicsToolboxSupportRegionDebug_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from KinematicsToolboxSupportRegionDebug_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit KinematicsToolboxSupportRegionDebug_.idl instead.
*
*/
public class KinematicsToolboxSupportRegionDebugPubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionDebug>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::KinematicsToolboxSupportRegionDebug_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "6b08d6dc8d235f9cc17ab1d64715c8e93ea111851f53b70b92d2f305cd332a9c";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionDebug data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionDebug data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 18; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionDebug data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionDebug data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getMultiContactFeasibleComRegion().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getMultiContactFeasibleComRegion().get(i0), current_alignment);}

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionDebug data, us.ihmc.idl.CDR cdr)
   {
      if(data.getMultiContactFeasibleComRegion().size() <= 18)
      cdr.write_type_e(data.getMultiContactFeasibleComRegion());else
          throw new RuntimeException("multi_contact_feasible_com_region field exceeds the maximum length");

      cdr.write_type_6(data.getCenterOfMassStabilityMargin());

   }

   public static void read(toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionDebug data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getMultiContactFeasibleComRegion());	
      data.setCenterOfMassStabilityMargin(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionDebug data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("multi_contact_feasible_com_region", data.getMultiContactFeasibleComRegion());
      ser.write_type_6("center_of_mass_stability_margin", data.getCenterOfMassStabilityMargin());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionDebug data)
   {
      ser.read_type_e("multi_contact_feasible_com_region", data.getMultiContactFeasibleComRegion());
      data.setCenterOfMassStabilityMargin(ser.read_type_6("center_of_mass_stability_margin"));
   }

   public static void staticCopy(toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionDebug src, toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionDebug dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionDebug createData()
   {
      return new toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionDebug();
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
   
   public void serialize(toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionDebug data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionDebug data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionDebug src, toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionDebug dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public KinematicsToolboxSupportRegionDebugPubSubType newInstance()
   {
      return new KinematicsToolboxSupportRegionDebugPubSubType();
   }
}
