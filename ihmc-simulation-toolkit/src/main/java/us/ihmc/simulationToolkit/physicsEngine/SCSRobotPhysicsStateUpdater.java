package us.ihmc.simulationToolkit.physicsEngine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyAccelerationProvider;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyTwistProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.robotics.physics.InertialMeasurementReader;
import us.ihmc.robotics.screwTheory.InvertedFourBarJoint;
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
   public void initialize(MultiBodySystemReadOnly multiBodySystem,
                          RigidBodyAccelerationProvider accelerationProvider,
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
      private final List<JointPhysicsStateUpdater> physicsStateUpdaters;

      public SingleRobotPhysicsStateUpdaters(RigidBodyReadOnly rootBody, Robot scsRobot)
      {
         physicsStateUpdaters = new ArrayList<>();

         for (JointReadOnly joint : rootBody.childrenSubtreeIterable())
         {
            if (joint instanceof InvertedFourBarJoint)
            {
               InvertedFourBarJoint fourBarJoint = (InvertedFourBarJoint) joint;
               physicsStateUpdaters.add(new InvertedFourBarJointPhysicsStateUpdater(fourBarJoint,
                                                                                    scsRobot.getJoint(fourBarJoint.getJointA().getName()),
                                                                                    scsRobot.getJoint(fourBarJoint.getJointB().getName()),
                                                                                    scsRobot.getJoint(fourBarJoint.getJointC().getName()),
                                                                                    scsRobot.getJoint(fourBarJoint.getJointD().getName())));
            }
            else
            {
               physicsStateUpdaters.add(new JointPhysicsStateUpdater(joint, scsRobot.getJoint(joint.getName())));
            }
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
         physicsStateUpdaters.forEach(JointPhysicsStateUpdater::initialize);
      }

      public void read(double dt, Vector3DReadOnly gravity)
      {
         physicsStateUpdaters.forEach(reader -> reader.read(dt, gravity));
      }
   }

   private class JointPhysicsStateUpdater
   {
      final RigidBodyReadOnly rigidBody;
      final Joint scsJoint;
      RigidBodyAccelerationProvider accelerationProvider;
      RigidBodyTwistProvider twistChangeProvider;

      ReferenceFrame inertialFrame;

      public JointPhysicsStateUpdater(JointReadOnly idJoint, Joint scsJoint)
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

   private class InvertedFourBarJointPhysicsStateUpdater extends JointPhysicsStateUpdater
   {
      private final InvertedFourBarJoint idFourBarJoint, idFourBarJointClone;
      private final Joint[] scsJoints;

      public InvertedFourBarJointPhysicsStateUpdater(InvertedFourBarJoint idJoint, Joint scsJointA, Joint scsJointB, Joint scsJointC, Joint scsJointD)
      {
         super(idJoint, null);
         idFourBarJoint = idJoint;
         scsJoints = new Joint[] {scsJointA, scsJointB, scsJointC, scsJointD};
         ReferenceFrame cloneFrame = ReferenceFrameTools.constructARootFrame("cloneStationaryFrame");
         idFourBarJointClone = InvertedFourBarJoint.cloneInvertedFourBarJoint(idFourBarJoint, cloneFrame, "clone");
      }

      @Override
      public void initialize()
      {
         for (int i = 0; i < 4; i++)
         {
            Joint scsJoint = scsJoints[i];

            if (scsJoint == null)
               continue;

            RevoluteJointBasics idJoint = idFourBarJoint.getFourBarFunction().getLoopJoints().get(i);

            ReferenceFrame frameAfterJoint = idJoint.getFrameAfterJoint();
            twist.setIncludingFrame(idJoint.getSuccessor().getBodyFixedFrame().getTwistOfFrame());

            scsJoint.jointTransform3D.set(frameAfterJoint.getTransformToParent());
            scsJoint.transformToNext.set(frameAfterJoint.getTransformToRoot());
            scsJoint.physics.Ri_0.setAndTranspose(scsJoint.transformToNext.getRotation());
            scsJoint.physics.w_i.set(twist.getAngularPart());
            scsJoint.physics.v_i.set(twist.getLinearPart());
            scsJoint.physics.a_hat_i.top.setToZero();
            scsJoint.physics.a_hat_i.bottom.setToZero();
         }
      }

      private final Twist twist = new Twist();
      private final SpatialAcceleration acceleration = new SpatialAcceleration();
      private final FrameVector3D localGravity = new FrameVector3D();
      private final SpatialAcceleration accelerationOfPredecessor = new SpatialAcceleration();
      private final SpatialAcceleration accelerationTemp = new SpatialAcceleration();
      private final SpatialAcceleration accelerationFromTwistChangeOfPredecessor = new SpatialAcceleration();
      private final Twist twistChangeOfJoint = new Twist();

      private final SpatialAcceleration[] loopJointAccelerations = {new SpatialAcceleration(), new SpatialAcceleration(), new SpatialAcceleration(),
            new SpatialAcceleration()};

      @Override
      public void read(double dt, Vector3DReadOnly gravity)
      {
         RigidBodyBasics predecessor = idFourBarJoint.getPredecessor();

         twistChangeOfJoint.setIncludingFrame(twistChangeProvider.getRelativeTwist(predecessor, rigidBody));
         twistChangeOfJoint.changeFrame(idFourBarJoint.getFrameAfterJoint());
         double qd_change = idFourBarJoint.getJointAxis().dot((Vector3DReadOnly) twistChangeOfJoint.getAngularPart());

         idFourBarJointClone.setQ(idFourBarJoint.getQ());
         idFourBarJointClone.setQd(idFourBarJoint.getQd());
         idFourBarJointClone.setQdd(idFourBarJoint.getQdd() + qd_change / dt);

         for (int i = 0; i < 4; i++)
         {
            RevoluteJointBasics idJoint = idFourBarJoint.getFourBarFunction().getLoopJoints().get(i);
            RevoluteJointBasics idJointClone = idFourBarJointClone.getFourBarFunction().getLoopJoints().get(i);
            SpatialAcceleration loopJointAcceleration = loopJointAccelerations[i];
            idJointClone.getSuccessorAcceleration(loopJointAcceleration);
            loopJointAcceleration.setBaseFrame(idJoint.getPredecessor().getBodyFixedFrame());
            loopJointAcceleration.setBodyFrame(idJoint.getSuccessor().getBodyFixedFrame());
            loopJointAcceleration.setReferenceFrame(idJoint.getSuccessor().getBodyFixedFrame());
         }

         accelerationTemp.setIncludingFrame(loopJointAccelerations[1]);
         accelerationTemp.changeFrame(loopJointAccelerations[2].getReferenceFrame());
         loopJointAccelerations[2].add(accelerationTemp);
         accelerationTemp.setIncludingFrame(loopJointAccelerations[0]);
         accelerationTemp.changeFrame(loopJointAccelerations[3].getReferenceFrame());
         loopJointAccelerations[3].add(accelerationTemp);

         accelerationOfPredecessor.setIncludingFrame(accelerationProvider.getAccelerationOfBody(predecessor));
         accelerationFromTwistChangeOfPredecessor.setIncludingFrame(twistChangeProvider.getTwistOfBody(predecessor));
         accelerationFromTwistChangeOfPredecessor.scale(1.0 / dt);
         accelerationOfPredecessor.add((SpatialVectorReadOnly) accelerationFromTwistChangeOfPredecessor);

         for (int i = 0; i < scsJoints.length; i++)
         {
            Joint scsJoint = scsJoints[i];

            if (scsJoint == null)
               continue;

            RevoluteJointBasics idJoint = idFourBarJoint.getFourBarFunction().getLoopJoints().get(i);

            ReferenceFrame frameAfterJoint = idJoint.getFrameAfterJoint();
            scsJoint.jointTransform3D.set(frameAfterJoint.getTransformToParent());
            scsJoint.transformToNext.set(frameAfterJoint.getTransformToRoot());
            scsJoint.physics.Ri_0.setAndTranspose(scsJoint.transformToNext.getRotation());

            twist.setIncludingFrame(idJoint.getSuccessor().getBodyFixedFrame().getTwistOfFrame());
            scsJoint.physics.w_i.set(twist.getAngularPart());
            scsJoint.physics.v_i.set(twist.getLinearPart());

            acceleration.setIncludingFrame(loopJointAccelerations[i]);
            acceleration.setBaseFrame(idFourBarJoint.getPredecessor().getBodyFixedFrame());
            acceleration.setBodyFrame(idJoint.getSuccessor().getBodyFixedFrame());
            acceleration.setReferenceFrame(idJoint.getSuccessor().getBodyFixedFrame());
            accelerationTemp.setIncludingFrame(accelerationOfPredecessor);
            accelerationTemp.changeFrame(idJoint.getSuccessor().getBodyFixedFrame());
            acceleration.add(accelerationTemp);

            scsJoint.physics.a_hat_i.top.set(acceleration.getAngularPart());
            scsJoint.physics.a_hat_i.bottom.set(acceleration.getLinearPart());

            localGravity.setIncludingFrame(inertialFrame, gravity);
            localGravity.changeFrame(idJoint.getSuccessor().getBodyFixedFrame());
            scsJoint.physics.a_hat_i.bottom.add(localGravity);
         }
      }
   }
}
