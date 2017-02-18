package us.ihmc.valkyrieRosControl.dataHolders;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.rosControl.wholeRobot.IMUHandle;


public class YoIMUHandleHolder
{
   private final IMUHandle handle;
   private final IMUDefinition imuDefinition;
   
   private final DoubleYoVariable xdd, ydd, zdd;
   private final DoubleYoVariable theta_x, theta_y, theta_z;
   private final DoubleYoVariable q_w, q_x, q_y, q_z;
   private final BooleanYoVariable isLinearAccelerationValid, isAngularRateValid, isOrientationMeasurementValid;

   public YoIMUHandleHolder(IMUHandle handle, IMUDefinition imuDefinition, YoVariableRegistry parentRegistry)
   {
      String name = handle.getName();
      YoVariableRegistry registry = new YoVariableRegistry(name);
      
      this.handle = handle;
      this.imuDefinition = imuDefinition;
      
      xdd = new DoubleYoVariable(name + "_xdd", registry);
      ydd = new DoubleYoVariable(name + "_ydd", registry);
      zdd = new DoubleYoVariable(name + "_zdd", registry);

      theta_x = new DoubleYoVariable(name + "_theta_x", registry);
      theta_y = new DoubleYoVariable(name + "_theta_y", registry);
      theta_z = new DoubleYoVariable(name + "_theta_z", registry);

      q_w = new DoubleYoVariable(name + "_q_w", registry);
      q_x = new DoubleYoVariable(name + "_q_x", registry);
      q_y = new DoubleYoVariable(name + "_q_y", registry);
      q_z = new DoubleYoVariable(name + "_q_z", registry);
      
      isLinearAccelerationValid = new BooleanYoVariable(name + "_isLinearAccelerationValid", registry);
      isAngularRateValid = new BooleanYoVariable(name + "_isAngularRateValid", registry);
      isOrientationMeasurementValid = new BooleanYoVariable(name + "_isOrientationMeasurementValid", registry);
      
      parentRegistry.addChild(registry);
      
   }
   
   public void update()
   {
      this.xdd.set(handle.getXdd());
      this.ydd.set(handle.getYdd());
      this.zdd.set(handle.getZdd());
      
      this.theta_x.set(handle.getTheta_x());
      this.theta_y.set(handle.getTheta_y());
      this.theta_z.set(handle.getTheta_z());
      
      this.q_w.set(handle.getQ_w());
      this.q_x.set(handle.getQ_x());
      this.q_y.set(handle.getQ_y());
      this.q_z.set(handle.getQ_z());
      
      if(handle instanceof MicroStrainIMUHandle) // Q_Q
      {
         this.isAngularRateValid.set(((MicroStrainIMUHandle) handle).isAngularRateValid());
         this.isLinearAccelerationValid.set(((MicroStrainIMUHandle) handle).isLinearAccelerationValid());
         this.isOrientationMeasurementValid.set(((MicroStrainIMUHandle) handle).isOrientationQuaternionValid());
      }
   }

   public IMUDefinition getImuDefinition()
   {
      return imuDefinition;
   }


   public void packLinearAcceleration(Vector3D accelerationToPack)
   {
      accelerationToPack.setX(this.xdd.getDoubleValue());
      accelerationToPack.setY(this.ydd.getDoubleValue());
      accelerationToPack.setZ(this.zdd.getDoubleValue());
   }
   
   public void packAngularVelocity(Vector3D angularVelocityToPack)
   {
      angularVelocityToPack.setX(this.theta_x.getDoubleValue());
      angularVelocityToPack.setY(this.theta_y.getDoubleValue());
      angularVelocityToPack.setZ(this.theta_z.getDoubleValue());
   }
   
   public void packOrientation(Quaternion orientationToPack)
   {
      orientationToPack.set(q_x.getDoubleValue(), q_y.getDoubleValue(), q_z.getDoubleValue(), q_w.getDoubleValue());
   }
   
}
