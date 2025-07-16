package exoskeleton_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is going to take in data and assign to each muscle channel
       */
public class ExoEMGMessage extends Packet<ExoEMGMessage> implements Settable<ExoEMGMessage>, EpsilonComparable<ExoEMGMessage>
{
   /**
            * message ID
            */
   public long id_;
   /**
            * floats for each muscle channel
            */
   public float frontalisrt_;
   public float frontalislt_;
   public float tart_;
   public float talt_;
   public float tprt_;
   public float tplt_;
   public float masseterrt_;
   public float masseterlt_;
   public float scmrt_;
   public float scmlt_;
   public float middeltrt_;
   public float middeltlt_;
   public float antdeltoidrt_;
   public float antdeltoidlt_;
   public float mserratusantrt_;
   public float mserratusantlt_;
   public float lattricepsrt_;
   public float lattricepslt_;
   public float brachiodrt_;
   public float brachiodlt_;
   public float bicepsbrrt_;
   public float bicepsbrlt_;
   public float flexcarpurt_;
   public float flexcarpult_;
   public float flexcarprrt_;
   public float flexcarprlt_;
   public float abductpolrt_;
   public float abductpollt_;
   public float extdigrt_;
   public float extdiglt_;
   public float pectmajorrt_;
   public float pectmajorlt_;
   public float rectabdomuprt_;
   public float rectabdomuplt_;
   public float rectabdomlort_;
   public float rectabdomlolt_;
   public float extobliquert_;
   public float extobliquelt_;
   public float intobliquert_;
   public float intobliquelt_;
   public float adductorsrt_;
   public float adductorslt_;
   public float soleusrt_;
   public float soleuslt_;
   public float tibantrt_;
   public float tibantlt_;
   public float peroneusrt_;
   public float peroneuslt_;
   public float medgastrort_;
   public float medgastrolt_;
   public float latgastrort_;
   public float latgastrolt_;
   public float bicepsfemrt_;
   public float bicepsfemlt_;
   public float semitendrt_;
   public float semitendlt_;
   public float vlort_;
   public float vlolt_;
   public float vmort_;
   public float vmolt_;
   public float rectusfemrt_;
   public float rectusfemlt_;
   public float pelvicfloorrt_;
   public float pelvicfloorlt_;
   public float uppertraprt_;
   public float uppertraplt_;
   public float middletraprt_;
   public float middletraplt_;
   public float lowertraprt_;
   public float lowertraplt_;
   public float thoracicrt_;
   public float thoraciclt_;
   public float multifidiirt_;
   public float multifidiilt_;
   public float glutmaxrt_;
   public float glutmaxlt_;
   public float glutmedrt_;
   public float glutmedlt_;
   public float latdorsirt_;
   public float latdorsilt_;
   public float infraspinrt_;
   public float infraspinlt_;
   public float cervicalpsrt_;
   public float cervicalpslt_;

   public ExoEMGMessage()
   {
   }

   public ExoEMGMessage(ExoEMGMessage other)
   {
      this();
      set(other);
   }

   public void set(ExoEMGMessage other)
   {
      id_ = other.id_;

      frontalisrt_ = other.frontalisrt_;

      frontalislt_ = other.frontalislt_;

      tart_ = other.tart_;

      talt_ = other.talt_;

      tprt_ = other.tprt_;

      tplt_ = other.tplt_;

      masseterrt_ = other.masseterrt_;

      masseterlt_ = other.masseterlt_;

      scmrt_ = other.scmrt_;

      scmlt_ = other.scmlt_;

      middeltrt_ = other.middeltrt_;

      middeltlt_ = other.middeltlt_;

      antdeltoidrt_ = other.antdeltoidrt_;

      antdeltoidlt_ = other.antdeltoidlt_;

      mserratusantrt_ = other.mserratusantrt_;

      mserratusantlt_ = other.mserratusantlt_;

      lattricepsrt_ = other.lattricepsrt_;

      lattricepslt_ = other.lattricepslt_;

      brachiodrt_ = other.brachiodrt_;

      brachiodlt_ = other.brachiodlt_;

      bicepsbrrt_ = other.bicepsbrrt_;

      bicepsbrlt_ = other.bicepsbrlt_;

      flexcarpurt_ = other.flexcarpurt_;

      flexcarpult_ = other.flexcarpult_;

      flexcarprrt_ = other.flexcarprrt_;

      flexcarprlt_ = other.flexcarprlt_;

      abductpolrt_ = other.abductpolrt_;

      abductpollt_ = other.abductpollt_;

      extdigrt_ = other.extdigrt_;

      extdiglt_ = other.extdiglt_;

      pectmajorrt_ = other.pectmajorrt_;

      pectmajorlt_ = other.pectmajorlt_;

      rectabdomuprt_ = other.rectabdomuprt_;

      rectabdomuplt_ = other.rectabdomuplt_;

      rectabdomlort_ = other.rectabdomlort_;

      rectabdomlolt_ = other.rectabdomlolt_;

      extobliquert_ = other.extobliquert_;

      extobliquelt_ = other.extobliquelt_;

      intobliquert_ = other.intobliquert_;

      intobliquelt_ = other.intobliquelt_;

      adductorsrt_ = other.adductorsrt_;

      adductorslt_ = other.adductorslt_;

      soleusrt_ = other.soleusrt_;

      soleuslt_ = other.soleuslt_;

      tibantrt_ = other.tibantrt_;

      tibantlt_ = other.tibantlt_;

      peroneusrt_ = other.peroneusrt_;

      peroneuslt_ = other.peroneuslt_;

      medgastrort_ = other.medgastrort_;

      medgastrolt_ = other.medgastrolt_;

      latgastrort_ = other.latgastrort_;

      latgastrolt_ = other.latgastrolt_;

      bicepsfemrt_ = other.bicepsfemrt_;

      bicepsfemlt_ = other.bicepsfemlt_;

      semitendrt_ = other.semitendrt_;

      semitendlt_ = other.semitendlt_;

      vlort_ = other.vlort_;

      vlolt_ = other.vlolt_;

      vmort_ = other.vmort_;

      vmolt_ = other.vmolt_;

      rectusfemrt_ = other.rectusfemrt_;

      rectusfemlt_ = other.rectusfemlt_;

      pelvicfloorrt_ = other.pelvicfloorrt_;

      pelvicfloorlt_ = other.pelvicfloorlt_;

      uppertraprt_ = other.uppertraprt_;

      uppertraplt_ = other.uppertraplt_;

      middletraprt_ = other.middletraprt_;

      middletraplt_ = other.middletraplt_;

      lowertraprt_ = other.lowertraprt_;

      lowertraplt_ = other.lowertraplt_;

      thoracicrt_ = other.thoracicrt_;

      thoraciclt_ = other.thoraciclt_;

      multifidiirt_ = other.multifidiirt_;

      multifidiilt_ = other.multifidiilt_;

      glutmaxrt_ = other.glutmaxrt_;

      glutmaxlt_ = other.glutmaxlt_;

      glutmedrt_ = other.glutmedrt_;

      glutmedlt_ = other.glutmedlt_;

      latdorsirt_ = other.latdorsirt_;

      latdorsilt_ = other.latdorsilt_;

      infraspinrt_ = other.infraspinrt_;

      infraspinlt_ = other.infraspinlt_;

      cervicalpsrt_ = other.cervicalpsrt_;

      cervicalpslt_ = other.cervicalpslt_;

   }

   /**
            * message ID
            */
   public void setId(long id)
   {
      id_ = id;
   }
   /**
            * message ID
            */
   public long getId()
   {
      return id_;
   }

   /**
            * floats for each muscle channel
            */
   public void setFrontalisrt(float frontalisrt)
   {
      frontalisrt_ = frontalisrt;
   }
   /**
            * floats for each muscle channel
            */
   public float getFrontalisrt()
   {
      return frontalisrt_;
   }

   public void setFrontalislt(float frontalislt)
   {
      frontalislt_ = frontalislt;
   }
   public float getFrontalislt()
   {
      return frontalislt_;
   }

   public void setTart(float tart)
   {
      tart_ = tart;
   }
   public float getTart()
   {
      return tart_;
   }

   public void setTalt(float talt)
   {
      talt_ = talt;
   }
   public float getTalt()
   {
      return talt_;
   }

   public void setTprt(float tprt)
   {
      tprt_ = tprt;
   }
   public float getTprt()
   {
      return tprt_;
   }

   public void setTplt(float tplt)
   {
      tplt_ = tplt;
   }
   public float getTplt()
   {
      return tplt_;
   }

   public void setMasseterrt(float masseterrt)
   {
      masseterrt_ = masseterrt;
   }
   public float getMasseterrt()
   {
      return masseterrt_;
   }

   public void setMasseterlt(float masseterlt)
   {
      masseterlt_ = masseterlt;
   }
   public float getMasseterlt()
   {
      return masseterlt_;
   }

   public void setScmrt(float scmrt)
   {
      scmrt_ = scmrt;
   }
   public float getScmrt()
   {
      return scmrt_;
   }

   public void setScmlt(float scmlt)
   {
      scmlt_ = scmlt;
   }
   public float getScmlt()
   {
      return scmlt_;
   }

   public void setMiddeltrt(float middeltrt)
   {
      middeltrt_ = middeltrt;
   }
   public float getMiddeltrt()
   {
      return middeltrt_;
   }

   public void setMiddeltlt(float middeltlt)
   {
      middeltlt_ = middeltlt;
   }
   public float getMiddeltlt()
   {
      return middeltlt_;
   }

   public void setAntdeltoidrt(float antdeltoidrt)
   {
      antdeltoidrt_ = antdeltoidrt;
   }
   public float getAntdeltoidrt()
   {
      return antdeltoidrt_;
   }

   public void setAntdeltoidlt(float antdeltoidlt)
   {
      antdeltoidlt_ = antdeltoidlt;
   }
   public float getAntdeltoidlt()
   {
      return antdeltoidlt_;
   }

   public void setMserratusantrt(float mserratusantrt)
   {
      mserratusantrt_ = mserratusantrt;
   }
   public float getMserratusantrt()
   {
      return mserratusantrt_;
   }

   public void setMserratusantlt(float mserratusantlt)
   {
      mserratusantlt_ = mserratusantlt;
   }
   public float getMserratusantlt()
   {
      return mserratusantlt_;
   }

   public void setLattricepsrt(float lattricepsrt)
   {
      lattricepsrt_ = lattricepsrt;
   }
   public float getLattricepsrt()
   {
      return lattricepsrt_;
   }

   public void setLattricepslt(float lattricepslt)
   {
      lattricepslt_ = lattricepslt;
   }
   public float getLattricepslt()
   {
      return lattricepslt_;
   }

   public void setBrachiodrt(float brachiodrt)
   {
      brachiodrt_ = brachiodrt;
   }
   public float getBrachiodrt()
   {
      return brachiodrt_;
   }

   public void setBrachiodlt(float brachiodlt)
   {
      brachiodlt_ = brachiodlt;
   }
   public float getBrachiodlt()
   {
      return brachiodlt_;
   }

   public void setBicepsbrrt(float bicepsbrrt)
   {
      bicepsbrrt_ = bicepsbrrt;
   }
   public float getBicepsbrrt()
   {
      return bicepsbrrt_;
   }

   public void setBicepsbrlt(float bicepsbrlt)
   {
      bicepsbrlt_ = bicepsbrlt;
   }
   public float getBicepsbrlt()
   {
      return bicepsbrlt_;
   }

   public void setFlexcarpurt(float flexcarpurt)
   {
      flexcarpurt_ = flexcarpurt;
   }
   public float getFlexcarpurt()
   {
      return flexcarpurt_;
   }

   public void setFlexcarpult(float flexcarpult)
   {
      flexcarpult_ = flexcarpult;
   }
   public float getFlexcarpult()
   {
      return flexcarpult_;
   }

   public void setFlexcarprrt(float flexcarprrt)
   {
      flexcarprrt_ = flexcarprrt;
   }
   public float getFlexcarprrt()
   {
      return flexcarprrt_;
   }

   public void setFlexcarprlt(float flexcarprlt)
   {
      flexcarprlt_ = flexcarprlt;
   }
   public float getFlexcarprlt()
   {
      return flexcarprlt_;
   }

   public void setAbductpolrt(float abductpolrt)
   {
      abductpolrt_ = abductpolrt;
   }
   public float getAbductpolrt()
   {
      return abductpolrt_;
   }

   public void setAbductpollt(float abductpollt)
   {
      abductpollt_ = abductpollt;
   }
   public float getAbductpollt()
   {
      return abductpollt_;
   }

   public void setExtdigrt(float extdigrt)
   {
      extdigrt_ = extdigrt;
   }
   public float getExtdigrt()
   {
      return extdigrt_;
   }

   public void setExtdiglt(float extdiglt)
   {
      extdiglt_ = extdiglt;
   }
   public float getExtdiglt()
   {
      return extdiglt_;
   }

   public void setPectmajorrt(float pectmajorrt)
   {
      pectmajorrt_ = pectmajorrt;
   }
   public float getPectmajorrt()
   {
      return pectmajorrt_;
   }

   public void setPectmajorlt(float pectmajorlt)
   {
      pectmajorlt_ = pectmajorlt;
   }
   public float getPectmajorlt()
   {
      return pectmajorlt_;
   }

   public void setRectabdomuprt(float rectabdomuprt)
   {
      rectabdomuprt_ = rectabdomuprt;
   }
   public float getRectabdomuprt()
   {
      return rectabdomuprt_;
   }

   public void setRectabdomuplt(float rectabdomuplt)
   {
      rectabdomuplt_ = rectabdomuplt;
   }
   public float getRectabdomuplt()
   {
      return rectabdomuplt_;
   }

   public void setRectabdomlort(float rectabdomlort)
   {
      rectabdomlort_ = rectabdomlort;
   }
   public float getRectabdomlort()
   {
      return rectabdomlort_;
   }

   public void setRectabdomlolt(float rectabdomlolt)
   {
      rectabdomlolt_ = rectabdomlolt;
   }
   public float getRectabdomlolt()
   {
      return rectabdomlolt_;
   }

   public void setExtobliquert(float extobliquert)
   {
      extobliquert_ = extobliquert;
   }
   public float getExtobliquert()
   {
      return extobliquert_;
   }

   public void setExtobliquelt(float extobliquelt)
   {
      extobliquelt_ = extobliquelt;
   }
   public float getExtobliquelt()
   {
      return extobliquelt_;
   }

   public void setIntobliquert(float intobliquert)
   {
      intobliquert_ = intobliquert;
   }
   public float getIntobliquert()
   {
      return intobliquert_;
   }

   public void setIntobliquelt(float intobliquelt)
   {
      intobliquelt_ = intobliquelt;
   }
   public float getIntobliquelt()
   {
      return intobliquelt_;
   }

   public void setAdductorsrt(float adductorsrt)
   {
      adductorsrt_ = adductorsrt;
   }
   public float getAdductorsrt()
   {
      return adductorsrt_;
   }

   public void setAdductorslt(float adductorslt)
   {
      adductorslt_ = adductorslt;
   }
   public float getAdductorslt()
   {
      return adductorslt_;
   }

   public void setSoleusrt(float soleusrt)
   {
      soleusrt_ = soleusrt;
   }
   public float getSoleusrt()
   {
      return soleusrt_;
   }

   public void setSoleuslt(float soleuslt)
   {
      soleuslt_ = soleuslt;
   }
   public float getSoleuslt()
   {
      return soleuslt_;
   }

   public void setTibantrt(float tibantrt)
   {
      tibantrt_ = tibantrt;
   }
   public float getTibantrt()
   {
      return tibantrt_;
   }

   public void setTibantlt(float tibantlt)
   {
      tibantlt_ = tibantlt;
   }
   public float getTibantlt()
   {
      return tibantlt_;
   }

   public void setPeroneusrt(float peroneusrt)
   {
      peroneusrt_ = peroneusrt;
   }
   public float getPeroneusrt()
   {
      return peroneusrt_;
   }

   public void setPeroneuslt(float peroneuslt)
   {
      peroneuslt_ = peroneuslt;
   }
   public float getPeroneuslt()
   {
      return peroneuslt_;
   }

   public void setMedgastrort(float medgastrort)
   {
      medgastrort_ = medgastrort;
   }
   public float getMedgastrort()
   {
      return medgastrort_;
   }

   public void setMedgastrolt(float medgastrolt)
   {
      medgastrolt_ = medgastrolt;
   }
   public float getMedgastrolt()
   {
      return medgastrolt_;
   }

   public void setLatgastrort(float latgastrort)
   {
      latgastrort_ = latgastrort;
   }
   public float getLatgastrort()
   {
      return latgastrort_;
   }

   public void setLatgastrolt(float latgastrolt)
   {
      latgastrolt_ = latgastrolt;
   }
   public float getLatgastrolt()
   {
      return latgastrolt_;
   }

   public void setBicepsfemrt(float bicepsfemrt)
   {
      bicepsfemrt_ = bicepsfemrt;
   }
   public float getBicepsfemrt()
   {
      return bicepsfemrt_;
   }

   public void setBicepsfemlt(float bicepsfemlt)
   {
      bicepsfemlt_ = bicepsfemlt;
   }
   public float getBicepsfemlt()
   {
      return bicepsfemlt_;
   }

   public void setSemitendrt(float semitendrt)
   {
      semitendrt_ = semitendrt;
   }
   public float getSemitendrt()
   {
      return semitendrt_;
   }

   public void setSemitendlt(float semitendlt)
   {
      semitendlt_ = semitendlt;
   }
   public float getSemitendlt()
   {
      return semitendlt_;
   }

   public void setVlort(float vlort)
   {
      vlort_ = vlort;
   }
   public float getVlort()
   {
      return vlort_;
   }

   public void setVlolt(float vlolt)
   {
      vlolt_ = vlolt;
   }
   public float getVlolt()
   {
      return vlolt_;
   }

   public void setVmort(float vmort)
   {
      vmort_ = vmort;
   }
   public float getVmort()
   {
      return vmort_;
   }

   public void setVmolt(float vmolt)
   {
      vmolt_ = vmolt;
   }
   public float getVmolt()
   {
      return vmolt_;
   }

   public void setRectusfemrt(float rectusfemrt)
   {
      rectusfemrt_ = rectusfemrt;
   }
   public float getRectusfemrt()
   {
      return rectusfemrt_;
   }

   public void setRectusfemlt(float rectusfemlt)
   {
      rectusfemlt_ = rectusfemlt;
   }
   public float getRectusfemlt()
   {
      return rectusfemlt_;
   }

   public void setPelvicfloorrt(float pelvicfloorrt)
   {
      pelvicfloorrt_ = pelvicfloorrt;
   }
   public float getPelvicfloorrt()
   {
      return pelvicfloorrt_;
   }

   public void setPelvicfloorlt(float pelvicfloorlt)
   {
      pelvicfloorlt_ = pelvicfloorlt;
   }
   public float getPelvicfloorlt()
   {
      return pelvicfloorlt_;
   }

   public void setUppertraprt(float uppertraprt)
   {
      uppertraprt_ = uppertraprt;
   }
   public float getUppertraprt()
   {
      return uppertraprt_;
   }

   public void setUppertraplt(float uppertraplt)
   {
      uppertraplt_ = uppertraplt;
   }
   public float getUppertraplt()
   {
      return uppertraplt_;
   }

   public void setMiddletraprt(float middletraprt)
   {
      middletraprt_ = middletraprt;
   }
   public float getMiddletraprt()
   {
      return middletraprt_;
   }

   public void setMiddletraplt(float middletraplt)
   {
      middletraplt_ = middletraplt;
   }
   public float getMiddletraplt()
   {
      return middletraplt_;
   }

   public void setLowertraprt(float lowertraprt)
   {
      lowertraprt_ = lowertraprt;
   }
   public float getLowertraprt()
   {
      return lowertraprt_;
   }

   public void setLowertraplt(float lowertraplt)
   {
      lowertraplt_ = lowertraplt;
   }
   public float getLowertraplt()
   {
      return lowertraplt_;
   }

   public void setThoracicrt(float thoracicrt)
   {
      thoracicrt_ = thoracicrt;
   }
   public float getThoracicrt()
   {
      return thoracicrt_;
   }

   public void setThoraciclt(float thoraciclt)
   {
      thoraciclt_ = thoraciclt;
   }
   public float getThoraciclt()
   {
      return thoraciclt_;
   }

   public void setMultifidiirt(float multifidiirt)
   {
      multifidiirt_ = multifidiirt;
   }
   public float getMultifidiirt()
   {
      return multifidiirt_;
   }

   public void setMultifidiilt(float multifidiilt)
   {
      multifidiilt_ = multifidiilt;
   }
   public float getMultifidiilt()
   {
      return multifidiilt_;
   }

   public void setGlutmaxrt(float glutmaxrt)
   {
      glutmaxrt_ = glutmaxrt;
   }
   public float getGlutmaxrt()
   {
      return glutmaxrt_;
   }

   public void setGlutmaxlt(float glutmaxlt)
   {
      glutmaxlt_ = glutmaxlt;
   }
   public float getGlutmaxlt()
   {
      return glutmaxlt_;
   }

   public void setGlutmedrt(float glutmedrt)
   {
      glutmedrt_ = glutmedrt;
   }
   public float getGlutmedrt()
   {
      return glutmedrt_;
   }

   public void setGlutmedlt(float glutmedlt)
   {
      glutmedlt_ = glutmedlt;
   }
   public float getGlutmedlt()
   {
      return glutmedlt_;
   }

   public void setLatdorsirt(float latdorsirt)
   {
      latdorsirt_ = latdorsirt;
   }
   public float getLatdorsirt()
   {
      return latdorsirt_;
   }

   public void setLatdorsilt(float latdorsilt)
   {
      latdorsilt_ = latdorsilt;
   }
   public float getLatdorsilt()
   {
      return latdorsilt_;
   }

   public void setInfraspinrt(float infraspinrt)
   {
      infraspinrt_ = infraspinrt;
   }
   public float getInfraspinrt()
   {
      return infraspinrt_;
   }

   public void setInfraspinlt(float infraspinlt)
   {
      infraspinlt_ = infraspinlt;
   }
   public float getInfraspinlt()
   {
      return infraspinlt_;
   }

   public void setCervicalpsrt(float cervicalpsrt)
   {
      cervicalpsrt_ = cervicalpsrt;
   }
   public float getCervicalpsrt()
   {
      return cervicalpsrt_;
   }

   public void setCervicalpslt(float cervicalpslt)
   {
      cervicalpslt_ = cervicalpslt;
   }
   public float getCervicalpslt()
   {
      return cervicalpslt_;
   }


   public static Supplier<ExoEMGMessagePubSubType> getPubSubType()
   {
      return ExoEMGMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ExoEMGMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ExoEMGMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.id_, other.id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.frontalisrt_, other.frontalisrt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.frontalislt_, other.frontalislt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.tart_, other.tart_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.talt_, other.talt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.tprt_, other.tprt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.tplt_, other.tplt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.masseterrt_, other.masseterrt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.masseterlt_, other.masseterlt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.scmrt_, other.scmrt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.scmlt_, other.scmlt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.middeltrt_, other.middeltrt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.middeltlt_, other.middeltlt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.antdeltoidrt_, other.antdeltoidrt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.antdeltoidlt_, other.antdeltoidlt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.mserratusantrt_, other.mserratusantrt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.mserratusantlt_, other.mserratusantlt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.lattricepsrt_, other.lattricepsrt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.lattricepslt_, other.lattricepslt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.brachiodrt_, other.brachiodrt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.brachiodlt_, other.brachiodlt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.bicepsbrrt_, other.bicepsbrrt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.bicepsbrlt_, other.bicepsbrlt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.flexcarpurt_, other.flexcarpurt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.flexcarpult_, other.flexcarpult_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.flexcarprrt_, other.flexcarprrt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.flexcarprlt_, other.flexcarprlt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.abductpolrt_, other.abductpolrt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.abductpollt_, other.abductpollt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.extdigrt_, other.extdigrt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.extdiglt_, other.extdiglt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.pectmajorrt_, other.pectmajorrt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.pectmajorlt_, other.pectmajorlt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.rectabdomuprt_, other.rectabdomuprt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.rectabdomuplt_, other.rectabdomuplt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.rectabdomlort_, other.rectabdomlort_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.rectabdomlolt_, other.rectabdomlolt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.extobliquert_, other.extobliquert_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.extobliquelt_, other.extobliquelt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.intobliquert_, other.intobliquert_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.intobliquelt_, other.intobliquelt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.adductorsrt_, other.adductorsrt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.adductorslt_, other.adductorslt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.soleusrt_, other.soleusrt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.soleuslt_, other.soleuslt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.tibantrt_, other.tibantrt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.tibantlt_, other.tibantlt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.peroneusrt_, other.peroneusrt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.peroneuslt_, other.peroneuslt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.medgastrort_, other.medgastrort_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.medgastrolt_, other.medgastrolt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.latgastrort_, other.latgastrort_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.latgastrolt_, other.latgastrolt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.bicepsfemrt_, other.bicepsfemrt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.bicepsfemlt_, other.bicepsfemlt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.semitendrt_, other.semitendrt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.semitendlt_, other.semitendlt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.vlort_, other.vlort_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.vlolt_, other.vlolt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.vmort_, other.vmort_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.vmolt_, other.vmolt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.rectusfemrt_, other.rectusfemrt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.rectusfemlt_, other.rectusfemlt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.pelvicfloorrt_, other.pelvicfloorrt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.pelvicfloorlt_, other.pelvicfloorlt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.uppertraprt_, other.uppertraprt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.uppertraplt_, other.uppertraplt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.middletraprt_, other.middletraprt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.middletraplt_, other.middletraplt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.lowertraprt_, other.lowertraprt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.lowertraplt_, other.lowertraplt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.thoracicrt_, other.thoracicrt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.thoraciclt_, other.thoraciclt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.multifidiirt_, other.multifidiirt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.multifidiilt_, other.multifidiilt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.glutmaxrt_, other.glutmaxrt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.glutmaxlt_, other.glutmaxlt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.glutmedrt_, other.glutmedrt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.glutmedlt_, other.glutmedlt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.latdorsirt_, other.latdorsirt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.latdorsilt_, other.latdorsilt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.infraspinrt_, other.infraspinrt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.infraspinlt_, other.infraspinlt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cervicalpsrt_, other.cervicalpsrt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cervicalpslt_, other.cervicalpslt_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ExoEMGMessage)) return false;

      ExoEMGMessage otherMyClass = (ExoEMGMessage) other;

      if(this.id_ != otherMyClass.id_) return false;

      if(this.frontalisrt_ != otherMyClass.frontalisrt_) return false;

      if(this.frontalislt_ != otherMyClass.frontalislt_) return false;

      if(this.tart_ != otherMyClass.tart_) return false;

      if(this.talt_ != otherMyClass.talt_) return false;

      if(this.tprt_ != otherMyClass.tprt_) return false;

      if(this.tplt_ != otherMyClass.tplt_) return false;

      if(this.masseterrt_ != otherMyClass.masseterrt_) return false;

      if(this.masseterlt_ != otherMyClass.masseterlt_) return false;

      if(this.scmrt_ != otherMyClass.scmrt_) return false;

      if(this.scmlt_ != otherMyClass.scmlt_) return false;

      if(this.middeltrt_ != otherMyClass.middeltrt_) return false;

      if(this.middeltlt_ != otherMyClass.middeltlt_) return false;

      if(this.antdeltoidrt_ != otherMyClass.antdeltoidrt_) return false;

      if(this.antdeltoidlt_ != otherMyClass.antdeltoidlt_) return false;

      if(this.mserratusantrt_ != otherMyClass.mserratusantrt_) return false;

      if(this.mserratusantlt_ != otherMyClass.mserratusantlt_) return false;

      if(this.lattricepsrt_ != otherMyClass.lattricepsrt_) return false;

      if(this.lattricepslt_ != otherMyClass.lattricepslt_) return false;

      if(this.brachiodrt_ != otherMyClass.brachiodrt_) return false;

      if(this.brachiodlt_ != otherMyClass.brachiodlt_) return false;

      if(this.bicepsbrrt_ != otherMyClass.bicepsbrrt_) return false;

      if(this.bicepsbrlt_ != otherMyClass.bicepsbrlt_) return false;

      if(this.flexcarpurt_ != otherMyClass.flexcarpurt_) return false;

      if(this.flexcarpult_ != otherMyClass.flexcarpult_) return false;

      if(this.flexcarprrt_ != otherMyClass.flexcarprrt_) return false;

      if(this.flexcarprlt_ != otherMyClass.flexcarprlt_) return false;

      if(this.abductpolrt_ != otherMyClass.abductpolrt_) return false;

      if(this.abductpollt_ != otherMyClass.abductpollt_) return false;

      if(this.extdigrt_ != otherMyClass.extdigrt_) return false;

      if(this.extdiglt_ != otherMyClass.extdiglt_) return false;

      if(this.pectmajorrt_ != otherMyClass.pectmajorrt_) return false;

      if(this.pectmajorlt_ != otherMyClass.pectmajorlt_) return false;

      if(this.rectabdomuprt_ != otherMyClass.rectabdomuprt_) return false;

      if(this.rectabdomuplt_ != otherMyClass.rectabdomuplt_) return false;

      if(this.rectabdomlort_ != otherMyClass.rectabdomlort_) return false;

      if(this.rectabdomlolt_ != otherMyClass.rectabdomlolt_) return false;

      if(this.extobliquert_ != otherMyClass.extobliquert_) return false;

      if(this.extobliquelt_ != otherMyClass.extobliquelt_) return false;

      if(this.intobliquert_ != otherMyClass.intobliquert_) return false;

      if(this.intobliquelt_ != otherMyClass.intobliquelt_) return false;

      if(this.adductorsrt_ != otherMyClass.adductorsrt_) return false;

      if(this.adductorslt_ != otherMyClass.adductorslt_) return false;

      if(this.soleusrt_ != otherMyClass.soleusrt_) return false;

      if(this.soleuslt_ != otherMyClass.soleuslt_) return false;

      if(this.tibantrt_ != otherMyClass.tibantrt_) return false;

      if(this.tibantlt_ != otherMyClass.tibantlt_) return false;

      if(this.peroneusrt_ != otherMyClass.peroneusrt_) return false;

      if(this.peroneuslt_ != otherMyClass.peroneuslt_) return false;

      if(this.medgastrort_ != otherMyClass.medgastrort_) return false;

      if(this.medgastrolt_ != otherMyClass.medgastrolt_) return false;

      if(this.latgastrort_ != otherMyClass.latgastrort_) return false;

      if(this.latgastrolt_ != otherMyClass.latgastrolt_) return false;

      if(this.bicepsfemrt_ != otherMyClass.bicepsfemrt_) return false;

      if(this.bicepsfemlt_ != otherMyClass.bicepsfemlt_) return false;

      if(this.semitendrt_ != otherMyClass.semitendrt_) return false;

      if(this.semitendlt_ != otherMyClass.semitendlt_) return false;

      if(this.vlort_ != otherMyClass.vlort_) return false;

      if(this.vlolt_ != otherMyClass.vlolt_) return false;

      if(this.vmort_ != otherMyClass.vmort_) return false;

      if(this.vmolt_ != otherMyClass.vmolt_) return false;

      if(this.rectusfemrt_ != otherMyClass.rectusfemrt_) return false;

      if(this.rectusfemlt_ != otherMyClass.rectusfemlt_) return false;

      if(this.pelvicfloorrt_ != otherMyClass.pelvicfloorrt_) return false;

      if(this.pelvicfloorlt_ != otherMyClass.pelvicfloorlt_) return false;

      if(this.uppertraprt_ != otherMyClass.uppertraprt_) return false;

      if(this.uppertraplt_ != otherMyClass.uppertraplt_) return false;

      if(this.middletraprt_ != otherMyClass.middletraprt_) return false;

      if(this.middletraplt_ != otherMyClass.middletraplt_) return false;

      if(this.lowertraprt_ != otherMyClass.lowertraprt_) return false;

      if(this.lowertraplt_ != otherMyClass.lowertraplt_) return false;

      if(this.thoracicrt_ != otherMyClass.thoracicrt_) return false;

      if(this.thoraciclt_ != otherMyClass.thoraciclt_) return false;

      if(this.multifidiirt_ != otherMyClass.multifidiirt_) return false;

      if(this.multifidiilt_ != otherMyClass.multifidiilt_) return false;

      if(this.glutmaxrt_ != otherMyClass.glutmaxrt_) return false;

      if(this.glutmaxlt_ != otherMyClass.glutmaxlt_) return false;

      if(this.glutmedrt_ != otherMyClass.glutmedrt_) return false;

      if(this.glutmedlt_ != otherMyClass.glutmedlt_) return false;

      if(this.latdorsirt_ != otherMyClass.latdorsirt_) return false;

      if(this.latdorsilt_ != otherMyClass.latdorsilt_) return false;

      if(this.infraspinrt_ != otherMyClass.infraspinrt_) return false;

      if(this.infraspinlt_ != otherMyClass.infraspinlt_) return false;

      if(this.cervicalpsrt_ != otherMyClass.cervicalpsrt_) return false;

      if(this.cervicalpslt_ != otherMyClass.cervicalpslt_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ExoEMGMessage {");
      builder.append("id=");
      builder.append(this.id_);      builder.append(", ");
      builder.append("frontalisrt=");
      builder.append(this.frontalisrt_);      builder.append(", ");
      builder.append("frontalislt=");
      builder.append(this.frontalislt_);      builder.append(", ");
      builder.append("tart=");
      builder.append(this.tart_);      builder.append(", ");
      builder.append("talt=");
      builder.append(this.talt_);      builder.append(", ");
      builder.append("tprt=");
      builder.append(this.tprt_);      builder.append(", ");
      builder.append("tplt=");
      builder.append(this.tplt_);      builder.append(", ");
      builder.append("masseterrt=");
      builder.append(this.masseterrt_);      builder.append(", ");
      builder.append("masseterlt=");
      builder.append(this.masseterlt_);      builder.append(", ");
      builder.append("scmrt=");
      builder.append(this.scmrt_);      builder.append(", ");
      builder.append("scmlt=");
      builder.append(this.scmlt_);      builder.append(", ");
      builder.append("middeltrt=");
      builder.append(this.middeltrt_);      builder.append(", ");
      builder.append("middeltlt=");
      builder.append(this.middeltlt_);      builder.append(", ");
      builder.append("antdeltoidrt=");
      builder.append(this.antdeltoidrt_);      builder.append(", ");
      builder.append("antdeltoidlt=");
      builder.append(this.antdeltoidlt_);      builder.append(", ");
      builder.append("mserratusantrt=");
      builder.append(this.mserratusantrt_);      builder.append(", ");
      builder.append("mserratusantlt=");
      builder.append(this.mserratusantlt_);      builder.append(", ");
      builder.append("lattricepsrt=");
      builder.append(this.lattricepsrt_);      builder.append(", ");
      builder.append("lattricepslt=");
      builder.append(this.lattricepslt_);      builder.append(", ");
      builder.append("brachiodrt=");
      builder.append(this.brachiodrt_);      builder.append(", ");
      builder.append("brachiodlt=");
      builder.append(this.brachiodlt_);      builder.append(", ");
      builder.append("bicepsbrrt=");
      builder.append(this.bicepsbrrt_);      builder.append(", ");
      builder.append("bicepsbrlt=");
      builder.append(this.bicepsbrlt_);      builder.append(", ");
      builder.append("flexcarpurt=");
      builder.append(this.flexcarpurt_);      builder.append(", ");
      builder.append("flexcarpult=");
      builder.append(this.flexcarpult_);      builder.append(", ");
      builder.append("flexcarprrt=");
      builder.append(this.flexcarprrt_);      builder.append(", ");
      builder.append("flexcarprlt=");
      builder.append(this.flexcarprlt_);      builder.append(", ");
      builder.append("abductpolrt=");
      builder.append(this.abductpolrt_);      builder.append(", ");
      builder.append("abductpollt=");
      builder.append(this.abductpollt_);      builder.append(", ");
      builder.append("extdigrt=");
      builder.append(this.extdigrt_);      builder.append(", ");
      builder.append("extdiglt=");
      builder.append(this.extdiglt_);      builder.append(", ");
      builder.append("pectmajorrt=");
      builder.append(this.pectmajorrt_);      builder.append(", ");
      builder.append("pectmajorlt=");
      builder.append(this.pectmajorlt_);      builder.append(", ");
      builder.append("rectabdomuprt=");
      builder.append(this.rectabdomuprt_);      builder.append(", ");
      builder.append("rectabdomuplt=");
      builder.append(this.rectabdomuplt_);      builder.append(", ");
      builder.append("rectabdomlort=");
      builder.append(this.rectabdomlort_);      builder.append(", ");
      builder.append("rectabdomlolt=");
      builder.append(this.rectabdomlolt_);      builder.append(", ");
      builder.append("extobliquert=");
      builder.append(this.extobliquert_);      builder.append(", ");
      builder.append("extobliquelt=");
      builder.append(this.extobliquelt_);      builder.append(", ");
      builder.append("intobliquert=");
      builder.append(this.intobliquert_);      builder.append(", ");
      builder.append("intobliquelt=");
      builder.append(this.intobliquelt_);      builder.append(", ");
      builder.append("adductorsrt=");
      builder.append(this.adductorsrt_);      builder.append(", ");
      builder.append("adductorslt=");
      builder.append(this.adductorslt_);      builder.append(", ");
      builder.append("soleusrt=");
      builder.append(this.soleusrt_);      builder.append(", ");
      builder.append("soleuslt=");
      builder.append(this.soleuslt_);      builder.append(", ");
      builder.append("tibantrt=");
      builder.append(this.tibantrt_);      builder.append(", ");
      builder.append("tibantlt=");
      builder.append(this.tibantlt_);      builder.append(", ");
      builder.append("peroneusrt=");
      builder.append(this.peroneusrt_);      builder.append(", ");
      builder.append("peroneuslt=");
      builder.append(this.peroneuslt_);      builder.append(", ");
      builder.append("medgastrort=");
      builder.append(this.medgastrort_);      builder.append(", ");
      builder.append("medgastrolt=");
      builder.append(this.medgastrolt_);      builder.append(", ");
      builder.append("latgastrort=");
      builder.append(this.latgastrort_);      builder.append(", ");
      builder.append("latgastrolt=");
      builder.append(this.latgastrolt_);      builder.append(", ");
      builder.append("bicepsfemrt=");
      builder.append(this.bicepsfemrt_);      builder.append(", ");
      builder.append("bicepsfemlt=");
      builder.append(this.bicepsfemlt_);      builder.append(", ");
      builder.append("semitendrt=");
      builder.append(this.semitendrt_);      builder.append(", ");
      builder.append("semitendlt=");
      builder.append(this.semitendlt_);      builder.append(", ");
      builder.append("vlort=");
      builder.append(this.vlort_);      builder.append(", ");
      builder.append("vlolt=");
      builder.append(this.vlolt_);      builder.append(", ");
      builder.append("vmort=");
      builder.append(this.vmort_);      builder.append(", ");
      builder.append("vmolt=");
      builder.append(this.vmolt_);      builder.append(", ");
      builder.append("rectusfemrt=");
      builder.append(this.rectusfemrt_);      builder.append(", ");
      builder.append("rectusfemlt=");
      builder.append(this.rectusfemlt_);      builder.append(", ");
      builder.append("pelvicfloorrt=");
      builder.append(this.pelvicfloorrt_);      builder.append(", ");
      builder.append("pelvicfloorlt=");
      builder.append(this.pelvicfloorlt_);      builder.append(", ");
      builder.append("uppertraprt=");
      builder.append(this.uppertraprt_);      builder.append(", ");
      builder.append("uppertraplt=");
      builder.append(this.uppertraplt_);      builder.append(", ");
      builder.append("middletraprt=");
      builder.append(this.middletraprt_);      builder.append(", ");
      builder.append("middletraplt=");
      builder.append(this.middletraplt_);      builder.append(", ");
      builder.append("lowertraprt=");
      builder.append(this.lowertraprt_);      builder.append(", ");
      builder.append("lowertraplt=");
      builder.append(this.lowertraplt_);      builder.append(", ");
      builder.append("thoracicrt=");
      builder.append(this.thoracicrt_);      builder.append(", ");
      builder.append("thoraciclt=");
      builder.append(this.thoraciclt_);      builder.append(", ");
      builder.append("multifidiirt=");
      builder.append(this.multifidiirt_);      builder.append(", ");
      builder.append("multifidiilt=");
      builder.append(this.multifidiilt_);      builder.append(", ");
      builder.append("glutmaxrt=");
      builder.append(this.glutmaxrt_);      builder.append(", ");
      builder.append("glutmaxlt=");
      builder.append(this.glutmaxlt_);      builder.append(", ");
      builder.append("glutmedrt=");
      builder.append(this.glutmedrt_);      builder.append(", ");
      builder.append("glutmedlt=");
      builder.append(this.glutmedlt_);      builder.append(", ");
      builder.append("latdorsirt=");
      builder.append(this.latdorsirt_);      builder.append(", ");
      builder.append("latdorsilt=");
      builder.append(this.latdorsilt_);      builder.append(", ");
      builder.append("infraspinrt=");
      builder.append(this.infraspinrt_);      builder.append(", ");
      builder.append("infraspinlt=");
      builder.append(this.infraspinlt_);      builder.append(", ");
      builder.append("cervicalpsrt=");
      builder.append(this.cervicalpsrt_);      builder.append(", ");
      builder.append("cervicalpslt=");
      builder.append(this.cervicalpslt_);
      builder.append("}");
      return builder.toString();
   }
}
