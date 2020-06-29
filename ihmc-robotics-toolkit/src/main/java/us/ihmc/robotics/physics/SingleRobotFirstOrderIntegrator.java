package us.ihmc.robotics.physics;

import java.util.List;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointBasics;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialAccelerationBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameTwistBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;

public class SingleRobotFirstOrderIntegrator
{
   private MultiBodySystemBasics input;
   private final DMatrixRMaj velocityChangeMatrix;

   /** Intermediate variable used to perform garbage-free operations. */
   private final Vector3D deltaPosition = new Vector3D();
   /** Intermediate variable used to perform garbage-free operations. */
   private final Vector3D rotationVector = new Vector3D();
   /** Intermediate variable used to perform garbage-free operations. */
   private final Quaternion orientationChange = new Quaternion();
   /** Intermediate variable used to perform garbage-free operations. */
   private final FrameVector3D linearAcceleration = new FrameVector3D();
   /** Intermediate variable used to perform garbage-free operations. */
   private final SpatialVector spatialVelocityChange = new SpatialVector();

   public SingleRobotFirstOrderIntegrator(MultiBodySystemBasics input)
   {
      this.input = input;
      velocityChangeMatrix = new DMatrixRMaj(MultiBodySystemTools.computeDegreesOfFreedom(input.getAllJoints()), 1);
   }

   public void resetJointVelocityChange()
   {
      velocityChangeMatrix.zero();
   }

   public void setJointVelocityChange(DMatrixRMaj velocityChange)
   {
      if (velocityChange == null)
         return;
      velocityChangeMatrix.set(velocityChange);
   }

   public void addJointVelocityChange(DMatrixRMaj velocityChange)
   {
      if (velocityChange == null)
         return;
      CommonOps_DDRM.addEquals(velocityChangeMatrix, velocityChange);
   }

   public void integrate(double dt)
   {
      List<? extends JointBasics> jointsToConsider = input.getJointsToConsider();
      int startIndex = 0;

      for (JointBasics joint : jointsToConsider)
      {
         if (joint instanceof OneDoFJointBasics)
         {
            double velocityChange = velocityChangeMatrix.get(startIndex);
            integrateOneDoFJoint(dt, (OneDoFJointBasics) joint, velocityChange);
         }
         else if (joint instanceof SixDoFJointBasics)
         {
            spatialVelocityChange.setIncludingFrame(joint.getFrameAfterJoint(), startIndex, velocityChangeMatrix);
            integrateFloatingJoint(dt, (SixDoFJointBasics) joint, spatialVelocityChange);
         }
         else
         {
            throw new UnsupportedOperationException("Unsupported joint " + joint);
         }
         startIndex += joint.getDegreesOfFreedom();
      }

      resetJointVelocityChange();
   }

   public void integrateOneDoFJoint(double dt, OneDoFJointBasics joint, double velocityChange)
   {
      double qdd = joint.getQdd() /* + velocityChange / dt */;
      double qd = joint.getQd() + qdd * dt + velocityChange;
      double q = joint.getQ() + (joint.getQd() + 0.5 * velocityChange) * dt + 0.5 * joint.getQdd() * dt * dt;
      joint.setQ(q);
      joint.setQd(qd);
      joint.setQdd(qdd);
   }

   public void integrateFloatingJoint(double dt, FloatingJointBasics joint, SpatialVectorReadOnly spatialVelocityChange)
   {
      FrameVector3DReadOnly angularVelocityChange = spatialVelocityChange.getAngularPart();
      FrameVector3DReadOnly linearVelocityChange = spatialVelocityChange.getLinearPart();

      Pose3DBasics pose = joint.getJointPose();
      QuaternionBasics orientation = pose.getOrientation();
      Point3DBasics position = pose.getPosition();

      FixedFrameTwistBasics twist = joint.getJointTwist();
      FixedFrameVector3DBasics angularVelocity = twist.getAngularPart();
      FixedFrameVector3DBasics linearVelocity = twist.getLinearPart();

      FixedFrameSpatialAccelerationBasics spatialAcceleration = joint.getJointAcceleration();
      FixedFrameVector3DBasics angularAcceleration = spatialAcceleration.getAngularPart();

      spatialAcceleration.getLinearAccelerationAtBodyOrigin(twist, linearAcceleration);

      rotationVector.setAndScale(dt, angularVelocity);
      rotationVector.scaleAdd(0.5 * dt, angularVelocityChange, rotationVector);
      rotationVector.scaleAdd(0.5 * dt * dt, angularAcceleration, rotationVector);
      orientationChange.setRotationVector(rotationVector);

      angularVelocity.scaleAdd(dt, angularAcceleration, angularVelocity);
      angularVelocity.add(angularVelocityChange);

      deltaPosition.setAndScale(dt, linearVelocity);
      deltaPosition.scaleAdd(0.5 * dt, linearVelocityChange, deltaPosition);
      deltaPosition.scaleAdd(0.5 * dt * dt, linearAcceleration, deltaPosition);
      orientation.transform(deltaPosition);
      position.add(deltaPosition);

      linearVelocity.scaleAdd(dt, linearAcceleration, linearVelocity);
      linearVelocity.add(linearVelocityChange);
      orientationChange.inverseTransform(linearVelocity);

      orientationChange.inverseTransform(linearAcceleration);
      spatialAcceleration.setBasedOnOriginAcceleration(angularAcceleration, linearAcceleration, twist);

      orientation.append(orientationChange);
   }
}
