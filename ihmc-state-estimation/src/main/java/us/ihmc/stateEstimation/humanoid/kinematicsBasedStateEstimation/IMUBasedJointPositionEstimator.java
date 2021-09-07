package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

/**
 * Calculates the position for a group of joints that connect two IMUs. Author: Jerry Pratt August
 * 3, 2021 Assumes that there are three joints between the parentIMU and the childIMU, they are yaw,
 * then roll, then pitch. Assumes that yaw is measured with a joint measurement device, like an
 * encoder, and provided. Assumes that the roll of the parentIMU link and the childIMU link is
 * always less than PI/2 (90 degrees) so that the IMU compass heading can be ignored. Once you get
 * past this angle, then pitch angle feeds into compass heading. Ignores IMU Compass since it is
 * going to be a poor reading and cannot be trusted. But one can trust very much that an IMU shows
 * which way down is. So as long as the roll of the IMU links are less than 90 degrees, we should be
 * able to deduce the pitch and roll of the joints between them.
 **/
public class IMUBasedJointPositionEstimator
{
   private final boolean DEBUG = false;
   private final IMUSensorReadOnly parentIMU;
   private final IMUSensorReadOnly childIMU;
   private final OneDoFJointBasics yawJoint;

   private final Quaternion parentJointToIMUQuaternion;
   private final Quaternion childJointToIMUQuaternion;

   private final CompassYawRemover compassYawRemover = new CompassYawRemover();

   private final Quaternion childIMUToWorldMeasured = new Quaternion();
   private final Quaternion parentIMUToWorldMeasured = new Quaternion();

   private final Quaternion jointYawToParentLink = new Quaternion();
   private final Quaternion jointYawToWorld = new Quaternion();
   private final Quaternion worldToJointYaw = new Quaternion();

   private final Quaternion childIMUToJointYaw = new Quaternion();
   private final Quaternion childLinkToJointYawCompassRemoved = new Quaternion();
   private final Tuple3DBasics eulerAnglesToPack = new Vector3D();

   private final Vector3D parentDownVector = new Vector3D();
   private final Vector3D childDownVector = new Vector3D();
   private final double[] rollAndPitchToPack = new double[2];

   private final YoDouble estimatedJointRoll, estimatedJointPitch, yawSolutionShouldBeZero;
   private final YoEnum<IMUBasedJointPositionEstimatorMode> mode;

   public enum IMUBasedJointPositionEstimatorMode
   {
      YawRollPitchUsingRotationMatrices, YawRollPitchUsingGravityDownVectors;
   }

   public IMUBasedJointPositionEstimator(String name, YoRegistry registry)
   {
      this(name, null, null, null, registry);
   }

   public void setIMUBasedJointPositionEstimatorMode(IMUBasedJointPositionEstimatorMode mode)
   {
      this.mode.set(mode);
   }

   public IMUBasedJointPositionEstimator(String name, OneDoFJointBasics yawJoint, IMUSensorReadOnly parentIMU, IMUSensorReadOnly childIMU, YoRegistry registry)
   {
      this.parentIMU = parentIMU;
      this.childIMU = childIMU;
      this.yawJoint = yawJoint;

      this.yawSolutionShouldBeZero = new YoDouble(name + "YawSolutionShouldBeZero", registry);
      this.estimatedJointRoll = new YoDouble(name + "EstimatedJointRoll", registry);
      this.estimatedJointPitch = new YoDouble(name + "EstimatedJointPitch", registry);

      this.mode = new YoEnum<IMUBasedJointPositionEstimatorMode>(name + "JointEstimatorMode",
                                                                 registry,
                                                                 IMUBasedJointPositionEstimatorMode.class); //
      mode.set(IMUBasedJointPositionEstimatorMode.YawRollPitchUsingGravityDownVectors);

      parentJointToIMUQuaternion = new Quaternion();
      childJointToIMUQuaternion = new Quaternion();

      if (parentIMU != null)
      {
         RigidBodyTransformReadOnly parentTransformFromIMUToJoint = parentIMU.getTransformFromIMUToJoint();
         parentTransformFromIMUToJoint.inverseTransform(parentJointToIMUQuaternion);

         RigidBodyTransformReadOnly childTransformFromIMUToJoint = childIMU.getTransformFromIMUToJoint();
         childTransformFromIMUToJoint.inverseTransform(childJointToIMUQuaternion);
      }
   }

   public void compute()
   {
      Quaternion parentIMUMeasurement = new Quaternion(parentIMU.getOrientationMeasurement());
      Quaternion childIMUMeasurement = new Quaternion(childIMU.getOrientationMeasurement());

      parentIMUMeasurement.append(parentJointToIMUQuaternion);
      childIMUMeasurement.append(childJointToIMUQuaternion);

      compute(yawJoint.getQ(), parentIMUMeasurement, childIMUMeasurement);
   }

   /**
    * Calculate the joint velocities and store results in YoVariables. If the first joint is yaw,
    * assumes that it is updated already from some other source, such as a joint encoder.
    */
   public void compute(double yaw, Orientation3DReadOnly parentIMUToWorld, Orientation3DReadOnly childIMUToWorld)
   {
      switch (mode.getValue())
      {
         case YawRollPitchUsingRotationMatrices:
            computeUsingRotationMatrices(yaw, parentIMUToWorld, childIMUToWorld);
            break;
         default:
         case YawRollPitchUsingGravityDownVectors:
            computeUsingGravityDownVectors(yaw, parentIMUToWorld, childIMUToWorld);
            break;
      }
   }

   private void computeUsingGravityDownVectors(double yaw, Orientation3DReadOnly parentIMUToWorld, Orientation3DReadOnly childIMUToWorld)
   {
      parentDownVector.set(0.0, 0.0, -1.0);
      childDownVector.set(0.0, 0.0, -1.0);

      parentIMUToWorld.inverseTransform(parentDownVector);
      jointYawToParentLink.setYawPitchRoll(yaw, 0.0, 0.0);
      jointYawToParentLink.inverseTransform(parentDownVector);

      childIMUToWorld.inverseTransform(childDownVector);

      compassYawRemover.computeRollAndPitchGivenTwoVectors(parentDownVector, childDownVector, rollAndPitchToPack);

      estimatedJointRoll.set(rollAndPitchToPack[0]);
      estimatedJointPitch.set(rollAndPitchToPack[1]);
      yawSolutionShouldBeZero.set(0.0);
   }

   private void computeUsingRotationMatrices(double yaw, Orientation3DReadOnly parentIMUToWorld, Orientation3DReadOnly childIMUToWorld)
   {
      parentIMUToWorldMeasured.set(parentIMUToWorld);
      childIMUToWorldMeasured.set(childIMUToWorld);

      if (DEBUG)
         System.out.println("parentIMUToWorldMeasured = " + parentIMUToWorldMeasured);
      if (DEBUG)
         System.out.println("childIMUToWorldMeasured = " + childIMUToWorldMeasured);

      jointYawToParentLink.setYawPitchRoll(yaw, 0.0, 0.0);

      jointYawToWorld.set(parentIMUToWorldMeasured);
      jointYawToWorld.multiply(jointYawToParentLink);

      if (DEBUG)
         System.out.println("jointYawToWorld = " + jointYawToWorld);

      worldToJointYaw.set(jointYawToWorld);
      worldToJointYaw.inverse();

      if (DEBUG)
         System.out.println("worldToJointYaw = " + worldToJointYaw);

      childIMUToJointYaw.set(worldToJointYaw);
      childIMUToJointYaw.multiply(childIMUToWorldMeasured);

      if (DEBUG)
         System.out.println("childIMUToJointYaw = " + childIMUToJointYaw);

      compassYawRemover.removeCompassYawFromMiddleAssumingRollPitchOrdering(worldToJointYaw, childIMUToWorld, childLinkToJointYawCompassRemoved);

      if (DEBUG)
         System.out.println("childLinkToJointYawCompassRemoved = " + childLinkToJointYawCompassRemoved);

      // Invert and take negative here since the joint is assumed to be in the order of yaw, roll, pitch.
      childLinkToJointYawCompassRemoved.inverse();

      childLinkToJointYawCompassRemoved.getEuler(eulerAnglesToPack);
      yawSolutionShouldBeZero.set(eulerAnglesToPack.getZ());
      estimatedJointRoll.set(-eulerAnglesToPack.getX());
      estimatedJointPitch.set(-eulerAnglesToPack.getY());

      if (DEBUG)
      {
         System.out.println("yawSolutionShouldBeZero = " + yawSolutionShouldBeZero.getValue());
         System.out.println("estimatedJointRoll = " + estimatedJointRoll.getValue());
         System.out.println("estimatedJointPitch = " + estimatedJointPitch.getValue());
      }
   }

   public double getEstimatedJointRoll()
   {
      return estimatedJointRoll.getValue();
   }

   public double getEstimatedJointPitch()
   {
      return estimatedJointPitch.getValue();
   }

   public double getYawSolutionShouldBeZero()
   {
      return yawSolutionShouldBeZero.getValue();
   }

}
