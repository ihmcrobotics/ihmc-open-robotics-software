package exoskeleton_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ExoEMGMessage" defined in "ExoEMGMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ExoEMGMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ExoEMGMessage_.idl instead.
*
*/
public class ExoEMGMessagePubSubType implements us.ihmc.pubsub.TopicDataType<exoskeleton_msgs.msg.dds.ExoEMGMessage>
{
   public static final java.lang.String name = "exoskeleton_msgs::msg::dds_::ExoEMGMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "5152896f70ec0b925094fbadbe75bb3c0edc6e71c9b63a22a13472fef9101149";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(exoskeleton_msgs.msg.dds.ExoEMGMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, exoskeleton_msgs.msg.dds.ExoEMGMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(exoskeleton_msgs.msg.dds.ExoEMGMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(exoskeleton_msgs.msg.dds.ExoEMGMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(exoskeleton_msgs.msg.dds.ExoEMGMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getId());

      cdr.write_type_5(data.getFrontalisrt());

      cdr.write_type_5(data.getFrontalislt());

      cdr.write_type_5(data.getTart());

      cdr.write_type_5(data.getTalt());

      cdr.write_type_5(data.getTprt());

      cdr.write_type_5(data.getTplt());

      cdr.write_type_5(data.getMasseterrt());

      cdr.write_type_5(data.getMasseterlt());

      cdr.write_type_5(data.getScmrt());

      cdr.write_type_5(data.getScmlt());

      cdr.write_type_5(data.getMiddeltrt());

      cdr.write_type_5(data.getMiddeltlt());

      cdr.write_type_5(data.getAntdeltoidrt());

      cdr.write_type_5(data.getAntdeltoidlt());

      cdr.write_type_5(data.getMserratusantrt());

      cdr.write_type_5(data.getMserratusantlt());

      cdr.write_type_5(data.getLattricepsrt());

      cdr.write_type_5(data.getLattricepslt());

      cdr.write_type_5(data.getBrachiodrt());

      cdr.write_type_5(data.getBrachiodlt());

      cdr.write_type_5(data.getBicepsbrrt());

      cdr.write_type_5(data.getBicepsbrlt());

      cdr.write_type_5(data.getFlexcarpurt());

      cdr.write_type_5(data.getFlexcarpult());

      cdr.write_type_5(data.getFlexcarprrt());

      cdr.write_type_5(data.getFlexcarprlt());

      cdr.write_type_5(data.getAbductpolrt());

      cdr.write_type_5(data.getAbductpollt());

      cdr.write_type_5(data.getExtdigrt());

      cdr.write_type_5(data.getExtdiglt());

      cdr.write_type_5(data.getPectmajorrt());

      cdr.write_type_5(data.getPectmajorlt());

      cdr.write_type_5(data.getRectabdomuprt());

      cdr.write_type_5(data.getRectabdomuplt());

      cdr.write_type_5(data.getRectabdomlort());

      cdr.write_type_5(data.getRectabdomlolt());

      cdr.write_type_5(data.getExtobliquert());

      cdr.write_type_5(data.getExtobliquelt());

      cdr.write_type_5(data.getIntobliquert());

      cdr.write_type_5(data.getIntobliquelt());

      cdr.write_type_5(data.getAdductorsrt());

      cdr.write_type_5(data.getAdductorslt());

      cdr.write_type_5(data.getSoleusrt());

      cdr.write_type_5(data.getSoleuslt());

      cdr.write_type_5(data.getTibantrt());

      cdr.write_type_5(data.getTibantlt());

      cdr.write_type_5(data.getPeroneusrt());

      cdr.write_type_5(data.getPeroneuslt());

      cdr.write_type_5(data.getMedgastrort());

      cdr.write_type_5(data.getMedgastrolt());

      cdr.write_type_5(data.getLatgastrort());

      cdr.write_type_5(data.getLatgastrolt());

      cdr.write_type_5(data.getBicepsfemrt());

      cdr.write_type_5(data.getBicepsfemlt());

      cdr.write_type_5(data.getSemitendrt());

      cdr.write_type_5(data.getSemitendlt());

      cdr.write_type_5(data.getVlort());

      cdr.write_type_5(data.getVlolt());

      cdr.write_type_5(data.getVmort());

      cdr.write_type_5(data.getVmolt());

      cdr.write_type_5(data.getRectusfemrt());

      cdr.write_type_5(data.getRectusfemlt());

      cdr.write_type_5(data.getPelvicfloorrt());

      cdr.write_type_5(data.getPelvicfloorlt());

      cdr.write_type_5(data.getUppertraprt());

      cdr.write_type_5(data.getUppertraplt());

      cdr.write_type_5(data.getMiddletraprt());

      cdr.write_type_5(data.getMiddletraplt());

      cdr.write_type_5(data.getLowertraprt());

      cdr.write_type_5(data.getLowertraplt());

      cdr.write_type_5(data.getThoracicrt());

      cdr.write_type_5(data.getThoraciclt());

      cdr.write_type_5(data.getMultifidiirt());

      cdr.write_type_5(data.getMultifidiilt());

      cdr.write_type_5(data.getGlutmaxrt());

      cdr.write_type_5(data.getGlutmaxlt());

      cdr.write_type_5(data.getGlutmedrt());

      cdr.write_type_5(data.getGlutmedlt());

      cdr.write_type_5(data.getLatdorsirt());

      cdr.write_type_5(data.getLatdorsilt());

      cdr.write_type_5(data.getInfraspinrt());

      cdr.write_type_5(data.getInfraspinlt());

      cdr.write_type_5(data.getCervicalpsrt());

      cdr.write_type_5(data.getCervicalpslt());

   }

   public static void read(exoskeleton_msgs.msg.dds.ExoEMGMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setId(cdr.read_type_4());
      	
      data.setFrontalisrt(cdr.read_type_5());
      	
      data.setFrontalislt(cdr.read_type_5());
      	
      data.setTart(cdr.read_type_5());
      	
      data.setTalt(cdr.read_type_5());
      	
      data.setTprt(cdr.read_type_5());
      	
      data.setTplt(cdr.read_type_5());
      	
      data.setMasseterrt(cdr.read_type_5());
      	
      data.setMasseterlt(cdr.read_type_5());
      	
      data.setScmrt(cdr.read_type_5());
      	
      data.setScmlt(cdr.read_type_5());
      	
      data.setMiddeltrt(cdr.read_type_5());
      	
      data.setMiddeltlt(cdr.read_type_5());
      	
      data.setAntdeltoidrt(cdr.read_type_5());
      	
      data.setAntdeltoidlt(cdr.read_type_5());
      	
      data.setMserratusantrt(cdr.read_type_5());
      	
      data.setMserratusantlt(cdr.read_type_5());
      	
      data.setLattricepsrt(cdr.read_type_5());
      	
      data.setLattricepslt(cdr.read_type_5());
      	
      data.setBrachiodrt(cdr.read_type_5());
      	
      data.setBrachiodlt(cdr.read_type_5());
      	
      data.setBicepsbrrt(cdr.read_type_5());
      	
      data.setBicepsbrlt(cdr.read_type_5());
      	
      data.setFlexcarpurt(cdr.read_type_5());
      	
      data.setFlexcarpult(cdr.read_type_5());
      	
      data.setFlexcarprrt(cdr.read_type_5());
      	
      data.setFlexcarprlt(cdr.read_type_5());
      	
      data.setAbductpolrt(cdr.read_type_5());
      	
      data.setAbductpollt(cdr.read_type_5());
      	
      data.setExtdigrt(cdr.read_type_5());
      	
      data.setExtdiglt(cdr.read_type_5());
      	
      data.setPectmajorrt(cdr.read_type_5());
      	
      data.setPectmajorlt(cdr.read_type_5());
      	
      data.setRectabdomuprt(cdr.read_type_5());
      	
      data.setRectabdomuplt(cdr.read_type_5());
      	
      data.setRectabdomlort(cdr.read_type_5());
      	
      data.setRectabdomlolt(cdr.read_type_5());
      	
      data.setExtobliquert(cdr.read_type_5());
      	
      data.setExtobliquelt(cdr.read_type_5());
      	
      data.setIntobliquert(cdr.read_type_5());
      	
      data.setIntobliquelt(cdr.read_type_5());
      	
      data.setAdductorsrt(cdr.read_type_5());
      	
      data.setAdductorslt(cdr.read_type_5());
      	
      data.setSoleusrt(cdr.read_type_5());
      	
      data.setSoleuslt(cdr.read_type_5());
      	
      data.setTibantrt(cdr.read_type_5());
      	
      data.setTibantlt(cdr.read_type_5());
      	
      data.setPeroneusrt(cdr.read_type_5());
      	
      data.setPeroneuslt(cdr.read_type_5());
      	
      data.setMedgastrort(cdr.read_type_5());
      	
      data.setMedgastrolt(cdr.read_type_5());
      	
      data.setLatgastrort(cdr.read_type_5());
      	
      data.setLatgastrolt(cdr.read_type_5());
      	
      data.setBicepsfemrt(cdr.read_type_5());
      	
      data.setBicepsfemlt(cdr.read_type_5());
      	
      data.setSemitendrt(cdr.read_type_5());
      	
      data.setSemitendlt(cdr.read_type_5());
      	
      data.setVlort(cdr.read_type_5());
      	
      data.setVlolt(cdr.read_type_5());
      	
      data.setVmort(cdr.read_type_5());
      	
      data.setVmolt(cdr.read_type_5());
      	
      data.setRectusfemrt(cdr.read_type_5());
      	
      data.setRectusfemlt(cdr.read_type_5());
      	
      data.setPelvicfloorrt(cdr.read_type_5());
      	
      data.setPelvicfloorlt(cdr.read_type_5());
      	
      data.setUppertraprt(cdr.read_type_5());
      	
      data.setUppertraplt(cdr.read_type_5());
      	
      data.setMiddletraprt(cdr.read_type_5());
      	
      data.setMiddletraplt(cdr.read_type_5());
      	
      data.setLowertraprt(cdr.read_type_5());
      	
      data.setLowertraplt(cdr.read_type_5());
      	
      data.setThoracicrt(cdr.read_type_5());
      	
      data.setThoraciclt(cdr.read_type_5());
      	
      data.setMultifidiirt(cdr.read_type_5());
      	
      data.setMultifidiilt(cdr.read_type_5());
      	
      data.setGlutmaxrt(cdr.read_type_5());
      	
      data.setGlutmaxlt(cdr.read_type_5());
      	
      data.setGlutmedrt(cdr.read_type_5());
      	
      data.setGlutmedlt(cdr.read_type_5());
      	
      data.setLatdorsirt(cdr.read_type_5());
      	
      data.setLatdorsilt(cdr.read_type_5());
      	
      data.setInfraspinrt(cdr.read_type_5());
      	
      data.setInfraspinlt(cdr.read_type_5());
      	
      data.setCervicalpsrt(cdr.read_type_5());
      	
      data.setCervicalpslt(cdr.read_type_5());
      	

   }

   @Override
   public final void serialize(exoskeleton_msgs.msg.dds.ExoEMGMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("id", data.getId());
      ser.write_type_5("frontalisrt", data.getFrontalisrt());
      ser.write_type_5("frontalislt", data.getFrontalislt());
      ser.write_type_5("tart", data.getTart());
      ser.write_type_5("talt", data.getTalt());
      ser.write_type_5("tprt", data.getTprt());
      ser.write_type_5("tplt", data.getTplt());
      ser.write_type_5("masseterrt", data.getMasseterrt());
      ser.write_type_5("masseterlt", data.getMasseterlt());
      ser.write_type_5("scmrt", data.getScmrt());
      ser.write_type_5("scmlt", data.getScmlt());
      ser.write_type_5("middeltrt", data.getMiddeltrt());
      ser.write_type_5("middeltlt", data.getMiddeltlt());
      ser.write_type_5("antdeltoidrt", data.getAntdeltoidrt());
      ser.write_type_5("antdeltoidlt", data.getAntdeltoidlt());
      ser.write_type_5("mserratusantrt", data.getMserratusantrt());
      ser.write_type_5("mserratusantlt", data.getMserratusantlt());
      ser.write_type_5("lattricepsrt", data.getLattricepsrt());
      ser.write_type_5("lattricepslt", data.getLattricepslt());
      ser.write_type_5("brachiodrt", data.getBrachiodrt());
      ser.write_type_5("brachiodlt", data.getBrachiodlt());
      ser.write_type_5("bicepsbrrt", data.getBicepsbrrt());
      ser.write_type_5("bicepsbrlt", data.getBicepsbrlt());
      ser.write_type_5("flexcarpurt", data.getFlexcarpurt());
      ser.write_type_5("flexcarpult", data.getFlexcarpult());
      ser.write_type_5("flexcarprrt", data.getFlexcarprrt());
      ser.write_type_5("flexcarprlt", data.getFlexcarprlt());
      ser.write_type_5("abductpolrt", data.getAbductpolrt());
      ser.write_type_5("abductpollt", data.getAbductpollt());
      ser.write_type_5("extdigrt", data.getExtdigrt());
      ser.write_type_5("extdiglt", data.getExtdiglt());
      ser.write_type_5("pectmajorrt", data.getPectmajorrt());
      ser.write_type_5("pectmajorlt", data.getPectmajorlt());
      ser.write_type_5("rectabdomuprt", data.getRectabdomuprt());
      ser.write_type_5("rectabdomuplt", data.getRectabdomuplt());
      ser.write_type_5("rectabdomlort", data.getRectabdomlort());
      ser.write_type_5("rectabdomlolt", data.getRectabdomlolt());
      ser.write_type_5("extobliquert", data.getExtobliquert());
      ser.write_type_5("extobliquelt", data.getExtobliquelt());
      ser.write_type_5("intobliquert", data.getIntobliquert());
      ser.write_type_5("intobliquelt", data.getIntobliquelt());
      ser.write_type_5("adductorsrt", data.getAdductorsrt());
      ser.write_type_5("adductorslt", data.getAdductorslt());
      ser.write_type_5("soleusrt", data.getSoleusrt());
      ser.write_type_5("soleuslt", data.getSoleuslt());
      ser.write_type_5("tibantrt", data.getTibantrt());
      ser.write_type_5("tibantlt", data.getTibantlt());
      ser.write_type_5("peroneusrt", data.getPeroneusrt());
      ser.write_type_5("peroneuslt", data.getPeroneuslt());
      ser.write_type_5("medgastrort", data.getMedgastrort());
      ser.write_type_5("medgastrolt", data.getMedgastrolt());
      ser.write_type_5("latgastrort", data.getLatgastrort());
      ser.write_type_5("latgastrolt", data.getLatgastrolt());
      ser.write_type_5("bicepsfemrt", data.getBicepsfemrt());
      ser.write_type_5("bicepsfemlt", data.getBicepsfemlt());
      ser.write_type_5("semitendrt", data.getSemitendrt());
      ser.write_type_5("semitendlt", data.getSemitendlt());
      ser.write_type_5("vlort", data.getVlort());
      ser.write_type_5("vlolt", data.getVlolt());
      ser.write_type_5("vmort", data.getVmort());
      ser.write_type_5("vmolt", data.getVmolt());
      ser.write_type_5("rectusfemrt", data.getRectusfemrt());
      ser.write_type_5("rectusfemlt", data.getRectusfemlt());
      ser.write_type_5("pelvicfloorrt", data.getPelvicfloorrt());
      ser.write_type_5("pelvicfloorlt", data.getPelvicfloorlt());
      ser.write_type_5("uppertraprt", data.getUppertraprt());
      ser.write_type_5("uppertraplt", data.getUppertraplt());
      ser.write_type_5("middletraprt", data.getMiddletraprt());
      ser.write_type_5("middletraplt", data.getMiddletraplt());
      ser.write_type_5("lowertraprt", data.getLowertraprt());
      ser.write_type_5("lowertraplt", data.getLowertraplt());
      ser.write_type_5("thoracicrt", data.getThoracicrt());
      ser.write_type_5("thoraciclt", data.getThoraciclt());
      ser.write_type_5("multifidiirt", data.getMultifidiirt());
      ser.write_type_5("multifidiilt", data.getMultifidiilt());
      ser.write_type_5("glutmaxrt", data.getGlutmaxrt());
      ser.write_type_5("glutmaxlt", data.getGlutmaxlt());
      ser.write_type_5("glutmedrt", data.getGlutmedrt());
      ser.write_type_5("glutmedlt", data.getGlutmedlt());
      ser.write_type_5("latdorsirt", data.getLatdorsirt());
      ser.write_type_5("latdorsilt", data.getLatdorsilt());
      ser.write_type_5("infraspinrt", data.getInfraspinrt());
      ser.write_type_5("infraspinlt", data.getInfraspinlt());
      ser.write_type_5("cervicalpsrt", data.getCervicalpsrt());
      ser.write_type_5("cervicalpslt", data.getCervicalpslt());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, exoskeleton_msgs.msg.dds.ExoEMGMessage data)
   {
      data.setId(ser.read_type_4("id"));
      data.setFrontalisrt(ser.read_type_5("frontalisrt"));
      data.setFrontalislt(ser.read_type_5("frontalislt"));
      data.setTart(ser.read_type_5("tart"));
      data.setTalt(ser.read_type_5("talt"));
      data.setTprt(ser.read_type_5("tprt"));
      data.setTplt(ser.read_type_5("tplt"));
      data.setMasseterrt(ser.read_type_5("masseterrt"));
      data.setMasseterlt(ser.read_type_5("masseterlt"));
      data.setScmrt(ser.read_type_5("scmrt"));
      data.setScmlt(ser.read_type_5("scmlt"));
      data.setMiddeltrt(ser.read_type_5("middeltrt"));
      data.setMiddeltlt(ser.read_type_5("middeltlt"));
      data.setAntdeltoidrt(ser.read_type_5("antdeltoidrt"));
      data.setAntdeltoidlt(ser.read_type_5("antdeltoidlt"));
      data.setMserratusantrt(ser.read_type_5("mserratusantrt"));
      data.setMserratusantlt(ser.read_type_5("mserratusantlt"));
      data.setLattricepsrt(ser.read_type_5("lattricepsrt"));
      data.setLattricepslt(ser.read_type_5("lattricepslt"));
      data.setBrachiodrt(ser.read_type_5("brachiodrt"));
      data.setBrachiodlt(ser.read_type_5("brachiodlt"));
      data.setBicepsbrrt(ser.read_type_5("bicepsbrrt"));
      data.setBicepsbrlt(ser.read_type_5("bicepsbrlt"));
      data.setFlexcarpurt(ser.read_type_5("flexcarpurt"));
      data.setFlexcarpult(ser.read_type_5("flexcarpult"));
      data.setFlexcarprrt(ser.read_type_5("flexcarprrt"));
      data.setFlexcarprlt(ser.read_type_5("flexcarprlt"));
      data.setAbductpolrt(ser.read_type_5("abductpolrt"));
      data.setAbductpollt(ser.read_type_5("abductpollt"));
      data.setExtdigrt(ser.read_type_5("extdigrt"));
      data.setExtdiglt(ser.read_type_5("extdiglt"));
      data.setPectmajorrt(ser.read_type_5("pectmajorrt"));
      data.setPectmajorlt(ser.read_type_5("pectmajorlt"));
      data.setRectabdomuprt(ser.read_type_5("rectabdomuprt"));
      data.setRectabdomuplt(ser.read_type_5("rectabdomuplt"));
      data.setRectabdomlort(ser.read_type_5("rectabdomlort"));
      data.setRectabdomlolt(ser.read_type_5("rectabdomlolt"));
      data.setExtobliquert(ser.read_type_5("extobliquert"));
      data.setExtobliquelt(ser.read_type_5("extobliquelt"));
      data.setIntobliquert(ser.read_type_5("intobliquert"));
      data.setIntobliquelt(ser.read_type_5("intobliquelt"));
      data.setAdductorsrt(ser.read_type_5("adductorsrt"));
      data.setAdductorslt(ser.read_type_5("adductorslt"));
      data.setSoleusrt(ser.read_type_5("soleusrt"));
      data.setSoleuslt(ser.read_type_5("soleuslt"));
      data.setTibantrt(ser.read_type_5("tibantrt"));
      data.setTibantlt(ser.read_type_5("tibantlt"));
      data.setPeroneusrt(ser.read_type_5("peroneusrt"));
      data.setPeroneuslt(ser.read_type_5("peroneuslt"));
      data.setMedgastrort(ser.read_type_5("medgastrort"));
      data.setMedgastrolt(ser.read_type_5("medgastrolt"));
      data.setLatgastrort(ser.read_type_5("latgastrort"));
      data.setLatgastrolt(ser.read_type_5("latgastrolt"));
      data.setBicepsfemrt(ser.read_type_5("bicepsfemrt"));
      data.setBicepsfemlt(ser.read_type_5("bicepsfemlt"));
      data.setSemitendrt(ser.read_type_5("semitendrt"));
      data.setSemitendlt(ser.read_type_5("semitendlt"));
      data.setVlort(ser.read_type_5("vlort"));
      data.setVlolt(ser.read_type_5("vlolt"));
      data.setVmort(ser.read_type_5("vmort"));
      data.setVmolt(ser.read_type_5("vmolt"));
      data.setRectusfemrt(ser.read_type_5("rectusfemrt"));
      data.setRectusfemlt(ser.read_type_5("rectusfemlt"));
      data.setPelvicfloorrt(ser.read_type_5("pelvicfloorrt"));
      data.setPelvicfloorlt(ser.read_type_5("pelvicfloorlt"));
      data.setUppertraprt(ser.read_type_5("uppertraprt"));
      data.setUppertraplt(ser.read_type_5("uppertraplt"));
      data.setMiddletraprt(ser.read_type_5("middletraprt"));
      data.setMiddletraplt(ser.read_type_5("middletraplt"));
      data.setLowertraprt(ser.read_type_5("lowertraprt"));
      data.setLowertraplt(ser.read_type_5("lowertraplt"));
      data.setThoracicrt(ser.read_type_5("thoracicrt"));
      data.setThoraciclt(ser.read_type_5("thoraciclt"));
      data.setMultifidiirt(ser.read_type_5("multifidiirt"));
      data.setMultifidiilt(ser.read_type_5("multifidiilt"));
      data.setGlutmaxrt(ser.read_type_5("glutmaxrt"));
      data.setGlutmaxlt(ser.read_type_5("glutmaxlt"));
      data.setGlutmedrt(ser.read_type_5("glutmedrt"));
      data.setGlutmedlt(ser.read_type_5("glutmedlt"));
      data.setLatdorsirt(ser.read_type_5("latdorsirt"));
      data.setLatdorsilt(ser.read_type_5("latdorsilt"));
      data.setInfraspinrt(ser.read_type_5("infraspinrt"));
      data.setInfraspinlt(ser.read_type_5("infraspinlt"));
      data.setCervicalpsrt(ser.read_type_5("cervicalpsrt"));
      data.setCervicalpslt(ser.read_type_5("cervicalpslt"));
   }

   public static void staticCopy(exoskeleton_msgs.msg.dds.ExoEMGMessage src, exoskeleton_msgs.msg.dds.ExoEMGMessage dest)
   {
      dest.set(src);
   }

   @Override
   public exoskeleton_msgs.msg.dds.ExoEMGMessage createData()
   {
      return new exoskeleton_msgs.msg.dds.ExoEMGMessage();
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
   
   public void serialize(exoskeleton_msgs.msg.dds.ExoEMGMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(exoskeleton_msgs.msg.dds.ExoEMGMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(exoskeleton_msgs.msg.dds.ExoEMGMessage src, exoskeleton_msgs.msg.dds.ExoEMGMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ExoEMGMessagePubSubType newInstance()
   {
      return new ExoEMGMessagePubSubType();
   }
}
