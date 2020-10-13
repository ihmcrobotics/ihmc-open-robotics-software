package us.ihmc.robotics.physics;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

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
import us.ihmc.mecano.algorithms.interfaces.RigidBodyTwistProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointMatrixIndexProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointReadOnly;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialAccelerationBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameTwistBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
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

   private final RigidBodyTwistChangeCalculator rigidBodyTwistChangeCalculator;
   private final RigidBodyTwistProvider rigidBodyTwistChangeProvider;

   public SingleRobotFirstOrderIntegrator(MultiBodySystemBasics input)
   {
      this.input = input;
      velocityChangeMatrix = new DMatrixRMaj(MultiBodySystemTools.computeDegreesOfFreedom(input.getAllJoints()), 1);
      rigidBodyTwistChangeCalculator = new RigidBodyTwistChangeCalculator();
      rigidBodyTwistChangeProvider = RigidBodyTwistProvider.toRigidBodyTwistProvider(rigidBodyTwistChangeCalculator, input.getInertialFrame());
   }

   public void reset()
   {
      velocityChangeMatrix.zero();
      rigidBodyTwistChangeCalculator.reset();
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

   public RigidBodyTwistProvider getRigidBodyTwistChangeProvider()
   {
      return rigidBodyTwistChangeProvider;
   }

   private class RigidBodyTwistChangeCalculator implements Function<RigidBodyReadOnly, TwistReadOnly>
   {
      public void reset()
      {
         rigidBodyTwistMap.clear();
      }

      private final Map<RigidBodyReadOnly, Twist> rigidBodyTwistMap = new HashMap<>();
      private final Twist jointTwist = new Twist();
      private final JointMatrixIndexProvider jointMatrixIndexProvider = input.getJointMatrixIndexProvider();

      @Override
      public TwistReadOnly apply(RigidBodyReadOnly body)
      {
         Twist twistOfBody = rigidBodyTwistMap.get(body);

         if (twistOfBody == null)
         {
            JointReadOnly parentJoint = body.getParentJoint();
            TwistReadOnly twistOfParentBody;
            RigidBodyReadOnly parentBody = parentJoint.getPredecessor();
            if (parentBody.isRootBody())
               twistOfParentBody = null;
            else
               twistOfParentBody = apply(parentBody);

            if (parentJoint instanceof OneDoFJointReadOnly)
            {
               jointTwist.setIncludingFrame(((OneDoFJointReadOnly) parentJoint).getUnitJointTwist());
               jointTwist.scale(velocityChangeMatrix.get(jointMatrixIndexProvider.getJointDoFIndices(parentJoint)[0]));
            }
            else if (parentJoint instanceof SixDoFJointReadOnly)
            {
               jointTwist.set(jointMatrixIndexProvider.getJointDoFIndices(parentJoint)[0], velocityChangeMatrix);
               jointTwist.setReferenceFrame(parentJoint.getFrameAfterJoint());
               jointTwist.setBaseFrame(parentJoint.getFrameBeforeJoint());
               jointTwist.setBodyFrame(parentJoint.getFrameAfterJoint());
            }
            else
            {
               throw new UnsupportedOperationException("Implement me for: " + parentJoint.getClass().getSimpleName());
            }

            jointTwist.changeFrame(body.getBodyFixedFrame());
            jointTwist.setBaseFrame(parentBody.getBodyFixedFrame());
            jointTwist.setBodyFrame(body.getBodyFixedFrame());

            twistOfBody = new Twist();
            if (twistOfParentBody == null)
               twistOfBody.setToZero(parentBody.getBodyFixedFrame(), input.getInertialFrame(), parentBody.getBodyFixedFrame());
            else
               twistOfBody.setIncludingFrame(twistOfParentBody);
            twistOfBody.changeFrame(body.getBodyFixedFrame());
            twistOfBody.add(jointTwist);
            rigidBodyTwistMap.put(body, twistOfBody);
         }

         return twistOfBody;
      }
   }
}
