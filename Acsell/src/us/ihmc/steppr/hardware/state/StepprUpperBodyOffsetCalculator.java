package us.ihmc.steppr.hardware.state;

import us.ihmc.acsell.hardware.state.AcsellActuatorState;
import us.ihmc.acsell.hardware.state.AcsellXSensState;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.steppr.hardware.configuration.StepprCalibrationOffset;

public class StepprUpperBodyOffsetCalculator
{
   // Values from matlab/fitHardIron.m
   private static final Vector3D magScale = new Vector3D(1.0 / 261.0, 1.0 / 243.0, 1.0 / 233.0);
   private static final Vector3D magBias = new Vector3D(-64.1, 54.0, -269.0);

   private static final RigidBodyTransform accelAndGyroToZUpMatrix = new RigidBodyTransform(new RotationMatrix(1, 0, 0, 0, 0, -1, 0, 1, 0), new Vector3D());
   private static final RigidBodyTransform magToAccellMatrix = new RigidBodyTransform(new RotationMatrix(0, 1, 0, 1, 0, 0, 0, 0, -1), new Vector3D());

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final AcsellActuatorState imuState;
   private final AcsellActuatorState torsoX;
   private final AcsellActuatorState torsoY;
   private final AcsellActuatorState torsoZ;

   private final AcsellXSensState xsens;

   private final Quaternion xsensQuat = new Quaternion();
   private final RotationMatrix xsensMatrix = new RotationMatrix();

   private final Vector3D accel = new Vector3D();
   private final Vector3D mag = new Vector3D();

   private final DoubleYoVariable q_calc_torso_x;
   private final DoubleYoVariable q_calc_torso_y;
   private final DoubleYoVariable q_calc_torso_z;

   private final AlphaFilteredYoVariable torso_yaw;
   private final AlphaFilteredYoVariable torso_pitch;
   private final AlphaFilteredYoVariable torso_roll;
   private final AlphaFilteredYoVariable xsens_yaw;
   private final AlphaFilteredYoVariable xsens_pitch;
   private final AlphaFilteredYoVariable xsens_roll;

   private final DoubleYoVariable pYawMagnet;

   private final DoubleYoVariable torsoMagX;
   private final DoubleYoVariable torsoMagY;
   private final DoubleYoVariable torsoMagZ;

   private final DoubleYoVariable torsoAccelX;
   private final DoubleYoVariable torsoAccelY;
   private final DoubleYoVariable torsoAccelZ;

   private double[] yawPitchRoll = new double[3];

   public StepprUpperBodyOffsetCalculator(AcsellActuatorState imuState, AcsellActuatorState backX, AcsellActuatorState backY, AcsellActuatorState backZ,
         AcsellXSensState xsens, double dt, YoVariableRegistry parentRegistry)
   {
      this.imuState = imuState;
      this.torsoX = backX;
      this.torsoY = backY;
      this.torsoZ = backZ;
      this.xsens = xsens;

      q_calc_torso_x = new DoubleYoVariable("q_calc_torso_x", registry);
      q_calc_torso_y = new DoubleYoVariable("q_calc_torso_y", registry);
      q_calc_torso_z = new DoubleYoVariable("q_calc_torso_z", registry);

      double alphaOrientation = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(0.1, dt);
      torso_yaw = new AlphaFilteredYoVariable("torso_yaw", registry, alphaOrientation);
      torso_pitch = new AlphaFilteredYoVariable("torso_pitch", registry, alphaOrientation);
      torso_roll = new AlphaFilteredYoVariable("torso_roll", registry, alphaOrientation);

      xsens_yaw = new AlphaFilteredYoVariable("xsens_yaw", registry, alphaOrientation);
      xsens_pitch = new AlphaFilteredYoVariable("xsens_pitch", registry, alphaOrientation);
      xsens_roll = new AlphaFilteredYoVariable("xsens_roll", registry, alphaOrientation);

      pYawMagnet = new DoubleYoVariable("pYawMagnet", registry);

      torsoMagX = new DoubleYoVariable("torsoMagX", registry);
      torsoMagY = new DoubleYoVariable("torsoMagY", registry);
      torsoMagZ = new DoubleYoVariable("torsoMagZ", registry);
      torsoAccelX = new DoubleYoVariable("torsoAccelX", registry);
      torsoAccelY = new DoubleYoVariable("torsoAccelY", registry);
      torsoAccelZ = new DoubleYoVariable("torsoAccelZ", registry);

      parentRegistry.addChild(registry);
   }

   public void accel2quaternions(Vector3D a, double heading)
   {
      double g = a.length();

      double roll = -Math.atan2(a.getY(), -a.getZ());
      if (!Double.isNaN(roll))
      {
         torso_roll.update(roll); // roll
      }

      double pitch = -Math.asin(a.getX() / -g);
      if (!Double.isNaN(pitch))
      {
         torso_pitch.update(pitch); // pitch
      }
      if (!Double.isNaN(heading))
      {
         torso_yaw.update(heading); // yaw
      }

   }

   public void update()
   {
      read();
      accel2quaternions(accel, pYawMagnet.getDoubleValue());

      xsens.getQuaternion(xsensQuat);
      xsensMatrix.set(xsensQuat);

      YawPitchRollConversion.convertQuaternionToYawPitchRoll(xsensQuat, yawPitchRoll);
      xsens_yaw.update(yawPitchRoll[0]);
      xsens_pitch.update(yawPitchRoll[1]);
      xsens_roll.update(yawPitchRoll[2]);
      q_calc_torso_x.set(torso_roll.getDoubleValue() - xsens_roll.getDoubleValue());
      q_calc_torso_y.set(torso_pitch.getDoubleValue() - xsens_pitch.getDoubleValue());
      q_calc_torso_z.set(0.0);

   }

   public void updateOffsets()
   {
      torsoX.updateCanonicalAngle(q_calc_torso_x.getDoubleValue() * 120.0, 2.0 * Math.PI, StepprCalibrationOffset.TORSO_X_MOTOR_ANGLE_OFFSET);
      torsoY.updateCanonicalAngle(q_calc_torso_y.getDoubleValue() * 120.0, 2.0 * Math.PI, StepprCalibrationOffset.TORSO_Y_MOTOR_ANGLE_OFFSET);
      torsoZ.updateCanonicalAngle(q_calc_torso_z.getDoubleValue() * 120.0, 2.0 * Math.PI, StepprCalibrationOffset.TORSO_Z_MOTOR_ANGLE_OFFSET);
   }

   private void read()
   {
      imuState.getIMUAccelerationVector(accel);
      imuState.getIMUMagnetoVector(mag);

      mag.setX((mag.getX() - magBias.getX()) * magScale.getX());
      mag.setY((mag.getY() - magBias.getY()) * magScale.getY());
      mag.setZ((mag.getZ() - magBias.getZ()) * magScale.getZ());

      accel.negate();
      accelAndGyroToZUpMatrix.transform(accel);

      magToAccellMatrix.transform(mag);
      accelAndGyroToZUpMatrix.transform(mag);

      torsoMagX.set(mag.getX());
      torsoMagY.set(mag.getY());
      torsoMagZ.set(mag.getZ());
      torsoAccelX.set(accel.getX());
      torsoAccelY.set(accel.getY());
      torsoAccelZ.set(accel.getZ());

      pYawMagnet.set(-Math.atan2(mag.getY(), mag.getX()));

   }

}
