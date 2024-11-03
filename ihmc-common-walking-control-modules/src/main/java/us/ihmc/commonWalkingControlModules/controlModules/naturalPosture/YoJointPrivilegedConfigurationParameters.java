package us.ihmc.commonWalkingControlModules.controlModules.naturalPosture;

import us.ihmc.commonWalkingControlModules.configurations.NaturalPostureParameters;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPDGains;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoJointPrivilegedConfigurationParameters
{
   private String jointName;
   private final YoDouble privilegedOrientation;
   private final YoPDGains pdGains;
   private final YoDouble weight;
   private final YoBoolean isPrimaryTask;
   private NaturalPostureParameters.OneDofJointPrivilegedParameters jointPrivilegedParameters;

   public YoJointPrivilegedConfigurationParameters(NaturalPostureParameters.OneDofJointPrivilegedParameters jointPrivilegedParameters, YoRegistry registry)
   {
      jointName = jointPrivilegedParameters.getJointName();
      String suffix = "_" + jointName.toLowerCase() + "_PrivilegedConfiguration";
      privilegedOrientation = new YoDouble("angle" + suffix, registry);
      pdGains = new YoPDGains(suffix, registry);
      weight = new YoDouble("weight" + suffix, registry);
      isPrimaryTask = new YoBoolean("isPrimaryTask" + suffix, registry);

      set(jointPrivilegedParameters);
   }

   public String getJointName()
   {
      return jointName;
   }

   public double getPrivilegedOrientation()
   {
      return privilegedOrientation.getDoubleValue();
   }

   public double getWeight()
   {
      return weight.getDoubleValue();
   }

   /**
    * This flag is TRUE when the privileged task should be considered primary and be added with the rest of the tasks in the QP. This flag is FALSE when the
    * privileged task is secondary and thus should be projected into the nullspace of the primary tasks (this is the usual case for privileged tasks).
    */
   public boolean isPrimaryTask()
   {
      return isPrimaryTask.getBooleanValue();
   }

   public YoDouble getYoPrivilegedOrientation()
   {
      return privilegedOrientation;
   }

   public YoPDGains getPDGains()
   {
      return pdGains;
   }

   public YoDouble getYoWeight()
   {
      return weight;
   }

   /**
    * This flag is TRUE when the privileged task should be considered primary and be added with the rest of the tasks in the QP. This flag is FALSE when the
    * privileged task is secondary and thus should be projected into the nullspace of the primary tasks (this is the usual case for privileged tasks).
    */
   public YoBoolean getYoIsPrimaryTask()
   {
      return isPrimaryTask;
   }

   public void set(YoJointPrivilegedConfigurationParameters other)
   {
      this.jointName = other.getJointName();
      this.privilegedOrientation.set(other.privilegedOrientation.getDoubleValue());
      this.pdGains.set(other.getPDGains());
      this.weight.set(other.weight.getDoubleValue());
      this.isPrimaryTask.set(other.getYoIsPrimaryTask().getBooleanValue());
   }

   private void set(NaturalPostureParameters.OneDofJointPrivilegedParameters parameters)
   {
      this.jointName = parameters.getJointName();
      this.privilegedOrientation.set(parameters.getPrivilegedOrientation());
      this.pdGains.setKp(parameters.getKp());
      this.pdGains.setKd(parameters.getKd());
      this.weight.set(parameters.getWeight());
      this.isPrimaryTask.set(parameters.isPrimaryTask());

      this.jointPrivilegedParameters = parameters;
   }

   public NaturalPostureParameters.OneDofJointPrivilegedParameters get()
   {
      return jointPrivilegedParameters;
   }
}