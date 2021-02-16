package us.ihmc.simulationToolkit.physicsEngine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyAccelerationProvider;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyTwistProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.robotics.physics.InertialMeasurementReader;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Robot;

public class SCSRobotPhysicsStateUpdater implements InertialMeasurementReader
{
   private final Map<RigidBodyReadOnly, SingleRobotPhysicsStateUpdaters> rootToRobotIMUSensorReadersMap = new HashMap<>();

   public void addRobot(RigidBodyReadOnly rootBody, Robot scsRobot)
   {
      rootToRobotIMUSensorReadersMap.put(rootBody, new SingleRobotPhysicsStateUpdaters(rootBody, scsRobot));
   }

   @Override
   public void initialize(MultiBodySystemReadOnly multiBodySystem, RigidBodyAccelerationProvider accelerationProvider,
                          RigidBodyTwistProvider twistChangeProvider)
   {
      SingleRobotPhysicsStateUpdaters singleRobotIMUSensorReaders = rootToRobotIMUSensorReadersMap.get(multiBodySystem.getRootBody());
      if (singleRobotIMUSensorReaders != null)
      {
         singleRobotIMUSensorReaders.setProviders(accelerationProvider, twistChangeProvider);
         singleRobotIMUSensorReaders.setInertialFrame(multiBodySystem.getInertialFrame());
         singleRobotIMUSensorReaders.initialize();
      }
   }

   @Override
   public void read(double dt, Vector3DReadOnly gravity)
   {
      rootToRobotIMUSensorReadersMap.values().forEach(readers -> readers.read(dt, gravity));
   }

   private class SingleRobotPhysicsStateUpdaters
   {
      private final List<PhysicsStateUpdater> physicsStateUpdaters;

      public SingleRobotPhysicsStateUpdaters(RigidBodyReadOnly rootBody, Robot scsRobot)
      {
         physicsStateUpdaters = new ArrayList<>();

         for (JointReadOnly joint : rootBody.childrenSubtreeIterable())
         {
            physicsStateUpdaters.add(new PhysicsStateUpdater(joint, scsRobot.getJoint(joint.getName())));
         }
      }

      public void setProviders(RigidBodyAccelerationProvider accelerationProvider, RigidBodyTwistProvider twistChangeProvider)
      {
         physicsStateUpdaters.forEach(reader -> reader.setProviders(accelerationProvider, twistChangeProvider));
      }

      public void setInertialFrame(ReferenceFrame inertialFrame)
      {
         physicsStateUpdaters.forEach(reader -> reader.setInertialFrame(inertialFrame));
      }

      public void initialize()
      {
         physicsStateUpdaters.forEach(PhysicsStateUpdater::initialize);
      }

      public void read(double dt, Vector3DReadOnly gravity)
      {
         physicsStateUpdaters.forEach(reader -> reader.read(dt, gravity));
      }
   }

   private class PhysicsStateUpdater
   {
      private final RigidBodyReadOnly rigidBody;
      private final Joint scsJoint;
      private RigidBodyAccelerationProvider accelerationProvider;
      private RigidBodyTwistProvider twistChangeProvider;

      private ReferenceFrame inertialFrame;

      public PhysicsStateUpdater(JointReadOnly idJoint, Joint scsJoint)
      {
         rigidBody = idJoint.getSuccessor();
         this.scsJoint = scsJoint;
      }

      public void setProviders(RigidBodyAccelerationProvider accelerationProvider, RigidBodyTwistProvider twistChangeProvider)
      {
         this.accelerationProvider = accelerationProvider;
         this.twistChangeProvider = twistChangeProvider;
      }

      public void setInertialFrame(ReferenceFrame inertialFrame)
      {
         this.inertialFrame = inertialFrame;
      }

      public void initialize()
      {
         ReferenceFrame frameAfterJoint = rigidBody.getParentJoint().getFrameAfterJoint();
         twist.setIncludingFrame(rigidBody.getBodyFixedFrame().getTwistOfFrame());

         scsJoint.jointTransform3D.set(frameAfterJoint.getTransformToParent());
         scsJoint.transformToNext.set(frameAfterJoint.getTransformToRoot());
         scsJoint.physics.Ri_0.setAndTranspose(scsJoint.transformToNext.getRotation());
         scsJoint.physics.w_i.set(twist.getAngularPart());
         scsJoint.physics.v_i.set(twist.getLinearPart());
         scsJoint.physics.a_hat_i.top.setToZero();
         scsJoint.physics.a_hat_i.bottom.setToZero();
      }

      private final Twist twist = new Twist();
      private final SpatialAcceleration acceleration = new SpatialAcceleration();
      private final FrameVector3D localGravity = new FrameVector3D();

      public void read(double dt, Vector3DReadOnly gravity)
      {
         SpatialAccelerationReadOnly accelerationOfBody = accelerationProvider.getAccelerationOfBody(rigidBody);
         TwistReadOnly twistChangeOfBody = twistChangeProvider.getTwistOfBody(rigidBody);

         twist.setIncludingFrame(rigidBody.getBodyFixedFrame().getTwistOfFrame());
         acceleration.setIncludingFrame(twistChangeOfBody);
         acceleration.scale(1.0 / dt);
         acceleration.add((SpatialVectorReadOnly) accelerationOfBody);

         ReferenceFrame frameAfterJoint = rigidBody.getParentJoint().getFrameAfterJoint();

         scsJoint.jointTransform3D.set(frameAfterJoint.getTransformToParent());
         scsJoint.transformToNext.set(frameAfterJoint.getTransformToRoot());
         scsJoint.physics.Ri_0.setAndTranspose(scsJoint.transformToNext.getRotation());
         scsJoint.physics.w_i.set(twist.getAngularPart());
         scsJoint.physics.v_i.set(twist.getLinearPart());
         scsJoint.physics.a_hat_i.top.set(acceleration.getAngularPart());
         scsJoint.physics.a_hat_i.bottom.set(acceleration.getLinearPart());

         localGravity.setIncludingFrame(inertialFrame, gravity);
         localGravity.changeFrame(rigidBody.getBodyFixedFrame());
         scsJoint.physics.a_hat_i.bottom.add(localGravity);
      }
   }
}
