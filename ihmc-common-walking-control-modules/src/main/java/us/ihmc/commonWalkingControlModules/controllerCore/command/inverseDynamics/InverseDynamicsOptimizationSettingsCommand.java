package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

/**
 * A command that can be used to configure the optimization settings inside the controller core. By
 * default the settings defined inside the {@link ControllerCoreOptimizationSettings} are used
 * inside the controller core. This command allows modifying some of these settings online using the
 * command API.
 *
 * <p>
 * TODO:<br>
 * Extend this command to allow for configuring more setting through the command API (see ticket
 * BEAST-973).
 * </p>
 *
 * @author Georg Wiedebach
 */
public class InverseDynamicsOptimizationSettingsCommand implements InverseDynamicsCommand<InverseDynamicsOptimizationSettingsCommand>
{
   /**
    * The lower bound on the rho values in the optimization.
    * 
    * @see ControllerCoreOptimizationSettings#getRhoMin()
    */
   private double rhoMin = Double.NaN;
   private double jointAccelerationMax = Double.NaN;
   private double rhoWeight = Double.NaN;
   private double rhoRateWeight = Double.NaN;
   private final Vector2D centerOfPressureWeight = new Vector2D(Double.NaN, Double.NaN);
   private final Vector2D centerOfPressureRateWeight = new Vector2D(Double.NaN, Double.NaN);
   private double jointAccelerationWeight = Double.NaN;
   private double jointJerkWeight = Double.NaN;
   private double jointTorqueWeight = Double.NaN;

   public void setRhoMin(double rhoMin)
   {
      this.rhoMin = rhoMin;
   }

   public void setJointAccelerationMax(double jointAccelerationMax)
   {
      this.jointAccelerationMax = jointAccelerationMax;
   }

   public void setRhoWeight(double rhoWeight)
   {
      this.rhoWeight = rhoWeight;
   }

   public void setRhoRateWeight(double rhoRateWeight)
   {
      this.rhoRateWeight = rhoRateWeight;
   }

   public void setCenterOfPressureWeight(Tuple2DReadOnly centerOfPressureWeight)
   {
      this.centerOfPressureWeight.set(centerOfPressureWeight);
   }

   public void setCenterOfPressureRateWeight(Tuple2DReadOnly centerOfPressureRateWeight)
   {
      this.centerOfPressureRateWeight.set(centerOfPressureRateWeight);
   }

   public void setJointAccelerationWeight(double jointAccelerationWeight)
   {
      this.jointAccelerationWeight = jointAccelerationWeight;
   }

   public void setJointJerkWeight(double jointJerkWeight)
   {
      this.jointJerkWeight = jointJerkWeight;
   }

   public void setJointTorqueWeight(double jointTorqueWeight)
   {
      this.jointTorqueWeight = jointTorqueWeight;
   }

   public boolean hasRhoMin()
   {
      return !Double.isNaN(rhoMin);
   }

   public boolean hasJointAccelerationMax()
   {
      return !Double.isNaN(jointAccelerationMax);
   }

   public boolean hasRhoWeight()
   {
      return !Double.isNaN(rhoWeight);
   }

   public boolean hasRhoRateWeight()
   {
      return !Double.isNaN(rhoRateWeight);
   }

   public boolean hasCenterOfPressureWeight()
   {
      return !centerOfPressureWeight.containsNaN();
   }

   public boolean hasCenterOfPressureRateWeight()
   {
      return !centerOfPressureRateWeight.containsNaN();
   }

   public boolean hasJointAccelerationWeight()
   {
      return !Double.isNaN(jointAccelerationWeight);
   }

   public boolean hasJointJerkWeight()
   {
      return !Double.isNaN(jointJerkWeight);
   }

   public boolean hasJointTorqueWeight()
   {
      return !Double.isNaN(jointTorqueWeight);
   }

   public double getRhoMin()
   {
      return rhoMin;
   }

   public double getJointAccelerationMax()
   {
      return jointAccelerationMax;
   }

   public double getRhoWeight()
   {
      return rhoWeight;
   }

   public double getRhoRateWeight()
   {
      return rhoRateWeight;
   }

   public Vector2D getCenterOfPressureWeight()
   {
      return centerOfPressureWeight;
   }

   public Vector2D getCenterOfPressureRateWeight()
   {
      return centerOfPressureRateWeight;
   }

   public double getJointAccelerationWeight()
   {
      return jointAccelerationWeight;
   }

   public double getJointJerkWeight()
   {
      return jointJerkWeight;
   }

   public double getJointTorqueWeight()
   {
      return jointTorqueWeight;
   }

   @Override
   public void set(InverseDynamicsOptimizationSettingsCommand other)
   {
      rhoMin = other.rhoMin;
      jointAccelerationMax = other.jointAccelerationMax;
      rhoWeight = other.rhoWeight;
      rhoRateWeight = other.rhoRateWeight;
      centerOfPressureWeight.set(other.centerOfPressureWeight);
      centerOfPressureRateWeight.set(other.centerOfPressureRateWeight);
      jointAccelerationWeight = other.jointAccelerationWeight;
      jointJerkWeight = other.jointJerkWeight;
      jointTorqueWeight = other.jointTorqueWeight;
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.OPTIMIZATION_SETTINGS;
   }
}
