package us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

public class VirtualModelControlOptimizationSettingsCommand implements InverseDynamicsCommand<VirtualModelControlOptimizationSettingsCommand>,
      InverseKinematicsCommand<VirtualModelControlOptimizationSettingsCommand>, VirtualModelControlCommand<VirtualModelControlOptimizationSettingsCommand>
{
   /**
    * The lower bound on the rho values in the optimization.
    * 
    * @see ControllerCoreOptimizationSettings#getRhoMin()
    */
   private double rhoMin = Double.NaN;
   private double rhoWeight = Double.NaN;
   private double rhoRateWeight = Double.NaN;
   private final Vector2D centerOfPressureWeight = new Vector2D(Double.NaN, Double.NaN);
   private final Vector2D centerOfPressureRateWeight = new Vector2D(Double.NaN, Double.NaN);
   private double momentumRateWeight = Double.NaN;
   private double momentumAccelerationWeight = Double.NaN;

   public void setRhoMin(double rhoMin)
   {
      this.rhoMin = rhoMin;
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

   public void setMomentumRateWeight(double momentumRateWeight)
   {
      this.momentumRateWeight = momentumRateWeight;
   }

   public void setMomentumAccelerationWeight(double momentumAccelerationWeight)
   {
      this.momentumAccelerationWeight = momentumAccelerationWeight;
   }

   public boolean hasRhoMin()
   {
      return !Double.isNaN(rhoMin);
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

   public boolean hasMomentumRateWeight()
   {
      return !Double.isNaN(momentumRateWeight);
   }

   public boolean hasMomentumAccelerationWeight()
   {
      return !Double.isNaN(momentumAccelerationWeight);
   }

   public double getRhoMin()
   {
      return rhoMin;
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

   public double getMomentumRateWeight()
   {
      return momentumRateWeight;
   }

   public double getMomentumAccelerationWeight()
   {
      return momentumAccelerationWeight;
   }

   @Override
   public void set(VirtualModelControlOptimizationSettingsCommand other)
   {
      rhoMin = other.rhoMin;
      rhoWeight = other.rhoWeight;
      rhoRateWeight = other.rhoRateWeight;
      centerOfPressureWeight.set(other.centerOfPressureWeight);
      centerOfPressureRateWeight.set(other.centerOfPressureRateWeight);
      momentumRateWeight = other.momentumRateWeight;
      momentumAccelerationWeight = other.momentumAccelerationWeight;
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.OPTIMIZATION_SETTINGS;
   }
}
