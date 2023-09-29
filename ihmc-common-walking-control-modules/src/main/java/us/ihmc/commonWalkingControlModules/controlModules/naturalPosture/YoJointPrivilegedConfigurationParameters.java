package us.ihmc.commonWalkingControlModules.controlModules.naturalPosture;

import us.ihmc.commonWalkingControlModules.configurations.NaturalPostureParameters;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoJointPrivilegedConfigurationParameters
{
   private final OneDoFJointBasics joint;
   private final YoDouble privilegedOrientation;
   private final YoDouble kp;
   private final YoDouble kd;
   private final YoDouble weight;
   private NaturalPostureParameters.OneDofJointPrivilegedParameters jointPrivilegedParameters;

   public YoJointPrivilegedConfigurationParameters(NaturalPostureParameters.OneDofJointPrivilegedParameters jointPrivilegedParameters, YoRegistry registry)
   {
      joint = jointPrivilegedParameters.getJoint();
      String suffix = "_" + joint.getName().toLowerCase() + "_PrivilegedConfiguration";
      privilegedOrientation = new YoDouble("angle" + suffix, registry);
      kp = new YoDouble("kp" + suffix, registry);
      kd = new YoDouble("kd" + suffix, registry);
      weight = new YoDouble("weight" + suffix, registry);

      set(jointPrivilegedParameters);
   }

   public void setPrivilegedOrientation(double privilegedOrientation)
   {
      this.privilegedOrientation.set(privilegedOrientation);
   }

   public void setKp(double kp)
   {
      this.kp.set(kp);
   }

   public void setKd(double kd)
   {
      this.kd.set(kd);
   }

   public void setWeight(double weight)
   {
      this.weight.set(weight);
   }

   public OneDoFJointBasics getJoint()
   {
      return joint;
   }

   public double getPrivilegedOrientation()
   {
      return privilegedOrientation.getDoubleValue();
   }

   public double getKp()
   {
      return kp.getDoubleValue();
   }

   public double getKd()
   {
      return kd.getDoubleValue();
   }

   public double getWeight()
   {
      return weight.getDoubleValue();
   }

   public YoDouble getYoPrivilegedOrientation()
   {
      return privilegedOrientation;
   }

   public YoDouble getYoKp()
   {
      return kp;
   }

   public YoDouble getYoKd()
   {
      return kd;
   }

   public YoDouble getYoWeight()
   {
      return weight;
   }

   public void set(YoJointPrivilegedConfigurationParameters other)
   {
      this.privilegedOrientation.set(other.privilegedOrientation.getDoubleValue());
      this.kp.set(other.kp.getDoubleValue());
      this.kd.set(other.kd.getDoubleValue());
      this.weight.set(other.weight.getDoubleValue());
   }

   private void set(NaturalPostureParameters.OneDofJointPrivilegedParameters parameters)
   {
      this.privilegedOrientation.set(parameters.getPrivilegedOrientation());
      this.kp.set(parameters.getKp());
      this.kd.set(parameters.getKd());
      this.weight.set(parameters.getWeight());

      this.jointPrivilegedParameters = parameters;
   }

   public NaturalPostureParameters.OneDofJointPrivilegedParameters get()
   {
      return jointPrivilegedParameters;
   }
}