package us.ihmc.simulationToolkit.physicsEngine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyAccelerationProvider;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyTwistProvider;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.robotics.physics.InertialMeasurementReader;
import us.ihmc.simulationconstructionset.IMUMount;
import us.ihmc.simulationconstructionset.Robot;

public class SCSRobotIMUSensorReader implements InertialMeasurementReader
{
   private final Map<RigidBodyReadOnly, SingleRobotIMUSensorReaders> rootToRobotIMUSensorReadersMap = new HashMap<>();

   public void addRobot(RigidBodyReadOnly rootBody, Robot scsRobot)
   {
      List<IMUMount> imuMounts = new ArrayList<>();
      scsRobot.getIMUMounts(imuMounts);
      rootToRobotIMUSensorReadersMap.put(rootBody, new SingleRobotIMUSensorReaders(rootBody, imuMounts));
   }

   @Override
   public void initialize(MultiBodySystemReadOnly multiBodySystem, RigidBodyAccelerationProvider accelerationProvider,
                          RigidBodyTwistProvider twistChangeProvider)
   {
      SingleRobotIMUSensorReaders singleRobotIMUSensorReaders = rootToRobotIMUSensorReadersMap.get(multiBodySystem.getRootBody());
      if (singleRobotIMUSensorReaders != null)
         singleRobotIMUSensorReaders.setProviders(accelerationProvider, twistChangeProvider);
   }

   @Override
   public void read(double dt)
   {
      rootToRobotIMUSensorReadersMap.values().forEach(readers -> readers.read(dt));
   }

   private class SingleRobotIMUSensorReaders
   {
      private final List<IMUMountReader> imuMountReaders;

      public SingleRobotIMUSensorReaders(RigidBodyReadOnly rootBody, List<IMUMount> imuMounts)
      {
         imuMountReaders = imuMounts.stream().map(imuMount -> new IMUMountReader(rootBody, imuMount)).collect(Collectors.toList());
      }

      public void setProviders(RigidBodyAccelerationProvider accelerationProvider, RigidBodyTwistProvider twistChangeProvider)
      {
         imuMountReaders.forEach(reader -> reader.setProviders(accelerationProvider, twistChangeProvider));
      }

      public void read(double dt)
      {
         imuMountReaders.forEach(reader -> reader.read(dt));
      }
   }

   private class IMUMountReader
   {
      private final RigidBodyReadOnly rigidBody;
      private final IMUMount imuMount;
      private RigidBodyAccelerationProvider accelerationProvider;
      private RigidBodyTwistProvider twistChangeProvider;

      private final MovingReferenceFrame sensorFrame;

      public IMUMountReader(RigidBodyReadOnly rootBody, IMUMount imuMount)
      {
         String parentJointName = imuMount.getParentJoint().getName();
         rigidBody = SubtreeStreams.fromChildren(rootBody).filter(joint -> joint.getName().equals(parentJointName)).findFirst().get().getSuccessor();
         this.imuMount = imuMount;
         RigidBodyTransform transformToParentJoint = new RigidBodyTransform();
         imuMount.getTransformFromMountToJoint(transformToParentJoint);
         sensorFrame = MovingReferenceFrame.constructFrameFixedInParent(imuMount.getName() + "SensorFrame",
                                                                        rigidBody.getParentJoint().getFrameAfterJoint(),
                                                                        transformToParentJoint);
      }

      public void setProviders(RigidBodyAccelerationProvider accelerationProvider, RigidBodyTwistProvider twistChangeProvider)
      {
         this.accelerationProvider = accelerationProvider;
         this.twistChangeProvider = twistChangeProvider;
      }

      private final Twist twist = new Twist();
      private final SpatialAcceleration acceleration = new SpatialAcceleration();
      private final FrameVector3D frameLinearAcceleration = new FrameVector3D();
      private final FramePoint3D sensorPosition = new FramePoint3D();

      private final Quaternion orientation = new Quaternion();
      private final Vector3D angularVelocityInBody = new Vector3D();
      private final Vector3D angularAccelerationInBody = new Vector3D();
      private final Vector3D linearAccelerationInBody = new Vector3D();

      public void read(double dt)
      {
         SpatialAccelerationReadOnly accelerationOfBody = accelerationProvider.getAccelerationOfBody(rigidBody);
         TwistReadOnly twistChangeOfBody = twistChangeProvider.getTwistOfBody(rigidBody);

         twist.setIncludingFrame(rigidBody.getBodyFixedFrame().getTwistOfFrame());
         acceleration.setIncludingFrame(twistChangeOfBody);
         acceleration.scale(1.0 / dt);
         acceleration.add((SpatialVectorReadOnly) accelerationOfBody);

         twist.changeFrame(sensorFrame);
//         acceleration.changeFrame(sensorFrame);

         orientation.set(sensorFrame.getTransformToRoot().getRotation());
         angularVelocityInBody.set(sensorFrame.getTwistOfFrame().getAngularPart());
         angularAccelerationInBody.set(acceleration.getAngularPart());
//         linearAccelerationInBody.set(acceleration.getLinearPart());
         sensorPosition.setToZero(sensorFrame);
         sensorPosition.changeFrame(rigidBody.getBodyFixedFrame());
         acceleration.getLinearAccelerationAt(rigidBody.getBodyFixedFrame().getTwistOfFrame(), sensorPosition, frameLinearAcceleration);
         frameLinearAcceleration.changeFrame(sensorFrame);
         linearAccelerationInBody.set(frameLinearAcceleration);

         imuMount.setOrientation(orientation);
         imuMount.setAngularVelocityInBody(angularVelocityInBody);
         imuMount.setAngularAccelerationInBody(angularAccelerationInBody);
         imuMount.setLinearAccelerationInBody(linearAccelerationInBody);
      }
   }
}
