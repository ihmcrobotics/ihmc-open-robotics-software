package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.Collection;
import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;

public class InverseDynamicsJointsFromSCSRobotGenerator
{
   private final SCSToInverseDynamicsJointMap scsToInverseDynamicsJointMap = new SCSToInverseDynamicsJointMap();

   private final RigidBodyBasics elevator;

   /**
    * Using the given SCS robot, generates an equivalent robot model using the
    * {@code InverseDynamicsJoint}s.
    * 
    * @param robot the simulated robot to use. Not modified.
    */
   //TODO: Add SliderJoints
   public InverseDynamicsJointsFromSCSRobotGenerator(Robot robot)
   {
      elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());

      ConcurrentLinkedQueue<Joint> jointQueue = new ConcurrentLinkedQueue<Joint>();
      jointQueue.addAll(robot.getRootJoints());

      while (!jointQueue.isEmpty())
      {
         Joint polledJoint = jointQueue.poll();

         // Link parameters:
         Link link = polledJoint.getLink();

         Matrix3D momentOfInertia = new Matrix3D();
         link.getMomentOfInertia(momentOfInertia);

         double mass = link.getMass();

         Vector3D comOffset = new Vector3D();
         link.getComOffset(comOffset);

         RigidBodyBasics parentIDBody = getParentIDBody(polledJoint, elevator);

         if (polledJoint instanceof FloatingJoint)
         {
            FloatingJoint currentJoint = (FloatingJoint) polledJoint;

            FloatingJointBasics currentIDJoint = new SixDoFJoint(currentJoint.getName(), elevator);
            new RigidBody(link.getName(), currentIDJoint, momentOfInertia, mass, comOffset);

            scsToInverseDynamicsJointMap.addLinkedJoints(currentJoint, currentIDJoint);
         }
         else if (polledJoint instanceof PinJoint)
         {
            PinJoint currentJoint = (PinJoint) polledJoint;

            Vector3D jointAxis = new Vector3D();
            currentJoint.getJointAxis(jointAxis);

            Vector3D jointOffset = new Vector3D();
            currentJoint.getOffset(jointOffset);

            RevoluteJoint currentIDJoint = new RevoluteJoint(currentJoint.getName(), parentIDBody, jointOffset, jointAxis);
            currentIDJoint.setJointLimitLower(currentJoint.getJointLowerLimit());
            currentIDJoint.setJointLimitUpper(currentJoint.getJointUpperLimit());

            new RigidBody(link.getName(), currentIDJoint, momentOfInertia, mass, comOffset);

            scsToInverseDynamicsJointMap.addLinkedJoints(currentJoint, currentIDJoint);
         }
         else if (polledJoint instanceof SliderJoint)
         {
            SliderJoint currentJoint = (SliderJoint) polledJoint;

            Vector3D jointAxis = new Vector3D();
            currentJoint.getJointAxis(jointAxis);

            Vector3D jointOffset = new Vector3D();
            currentJoint.getOffset(jointOffset);

            PrismaticJoint currentIDJoint = new PrismaticJoint(currentJoint.getName(), parentIDBody, jointOffset, jointAxis);
            currentIDJoint.setJointLimitLower(currentJoint.getJointLowerLimit());
            currentIDJoint.setJointLimitUpper(currentJoint.getJointUpperLimit());

            new RigidBody(link.getName(), currentIDJoint, momentOfInertia, mass, comOffset);

            scsToInverseDynamicsJointMap.addLinkedJoints(currentJoint, currentIDJoint);
         }
         else
         {
            throw new RuntimeException();
         }

         jointQueue.addAll(polledJoint.getChildrenJoints());
      }

      elevator.updateFramesRecursively();
   }

   /**
    * Gets map from the simulated robot joints to the {@code InverseDynamicsJoint}s.
    * 
    * @return the map from simulated joint to {@code InverseDynamicsJoint}s.
    */
   public SCSToInverseDynamicsJointMap getSCSToInverseDynamicsJointMap()
   {
      return scsToInverseDynamicsJointMap;
   }

   /**
    * Gets the root body to which is attached the very first {@code InverseDynamicsJoint}.
    * 
    * @return the elevator.
    */
   public RigidBodyBasics getElevator()
   {
      return elevator;
   }

   private RigidBodyBasics getParentIDBody(Joint polledJoint, RigidBodyBasics elevator)
   {
      Joint parentJoint = polledJoint.getParentJoint();

      if (parentJoint == null)
         return elevator;

      return scsToInverseDynamicsJointMap.getRigidBody(parentJoint);
   }

   private final SpatialAcceleration spatialAccelerationVector = new SpatialAcceleration();

   private final FrameVector3D linearVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final FrameVector3D angularVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame());

   private final FrameVector3D originAcceleration = new FrameVector3D();
   private final FrameVector3D angularAcceleration = new FrameVector3D();

   private final RigidBodyTransform positionAndRotation = new RigidBodyTransform();

   /**
    * Update the configuration state of the {@code InverseDynamicsJoint}s using the state of the
    * simulated joints.
    * 
    * @param updateRootJoints whether the state of the root joints should be updated or not.
    */
   public void updateInverseDynamicsRobotModelFromRobot(boolean updateRootJoints)
   {
      // First update joint angles:
      Collection<OneDegreeOfFreedomJoint> pinJoints = scsToInverseDynamicsJointMap.getSCSOneDegreeOfFreedomJoints();
      for (OneDegreeOfFreedomJoint pinJoint : pinJoints)
      {
         if (updateRootJoints || (pinJoint.getParentJoint() != null))
         {
            OneDoFJointBasics revoluteJoint = scsToInverseDynamicsJointMap.getInverseDynamicsOneDoFJoint(pinJoint);

            double jointPosition = pinJoint.getQYoVariable().getDoubleValue();
            double jointVelocity = pinJoint.getQDYoVariable().getDoubleValue();
            revoluteJoint.setQ(jointPosition);
            revoluteJoint.setQd(jointVelocity);
         }
      }

      if (updateRootJoints)
      {
         Collection<? extends FloatingJoint> floatingJoints = scsToInverseDynamicsJointMap.getFloatingJoints();
         for (FloatingJoint floatingJoint : floatingJoints)
         {
            FloatingJointBasics sixDoFJoint = scsToInverseDynamicsJointMap.getInverseDynamicsSixDoFJoint(floatingJoint);

            floatingJoint.getTransformToWorld(positionAndRotation);
            sixDoFJoint.setJointConfiguration(positionAndRotation);
         }
      }

      // Then update all the frames. They will be needed for further computation:
      elevator.updateFramesRecursively();

      if (updateRootJoints)
      {
         Collection<? extends FloatingJoint> floatingJoints = scsToInverseDynamicsJointMap.getFloatingJoints();
         for (FloatingJoint floatingJoint : floatingJoints)
         {
            FloatingJointBasics sixDoFJoint = scsToInverseDynamicsJointMap.getInverseDynamicsSixDoFJoint(floatingJoint);
            //                     referenceFrames.updateFrames();

            ReferenceFrame elevatorFrame = sixDoFJoint.getFrameBeforeJoint();
            ReferenceFrame pelvisFrame = sixDoFJoint.getFrameAfterJoint();

            floatingJoint.getVelocity(linearVelocity);
            linearVelocity.changeFrame(pelvisFrame);

            floatingJoint.getAngularVelocity(angularVelocity, pelvisFrame);

            Twist bodyTwist = new Twist(pelvisFrame, elevatorFrame, pelvisFrame, angularVelocity, linearVelocity);
            sixDoFJoint.setJointTwist(bodyTwist);
            sixDoFJoint.updateFramesRecursively();
         }
      }
   }

   /**
    * Updates an the state of the simulated robot with the state from the Inverse Dynamics Model.
    *
    * @param updateRootJoints whether the root joints' state should also be updated.
    * @param updatePositions whether the joint positions should be copied over to the SCS robot or
    *           not.
    * @param updateVelocities whether the joint velocities should be copied over to the SCS robot or
    *           not.
    * @param updateAccelerations whether the joint accelerations should be copied over to the SCS
    *           robot or not.
    * @param updateTorques whether the joint forces/torques should be copied over to the SCS robot
    *           or not.
    */
   public void updateRobotFromInverseDynamicsRobotModel(boolean updateRootJoints, boolean updatePositions, boolean updateVelocities,
                                                        boolean updateAccelerations, boolean updateTorques)
   {

      Collection<OneDegreeOfFreedomJoint> pinJoints = scsToInverseDynamicsJointMap.getSCSOneDegreeOfFreedomJoints();
      for (OneDegreeOfFreedomJoint pinJoint : pinJoints)
      {
         if (updateRootJoints || (pinJoint.getParentJoint() != null))
         {
            OneDoFJointBasics revoluteJoint = scsToInverseDynamicsJointMap.getInverseDynamicsOneDoFJoint(pinJoint);

            double jointPosition;
            double jointVelocity;
            double jointAcceleration;
            double jointTorque;

            if (updatePositions)
            {
               jointPosition = revoluteJoint.getQ();
               pinJoint.setQ(jointPosition);
            }

            if (updateAccelerations)
            {
               jointAcceleration = revoluteJoint.getQdd();
               pinJoint.setQdd(jointAcceleration);
            }

            if (updateVelocities)
            {
               jointVelocity = revoluteJoint.getQd();
               pinJoint.setQd(jointVelocity);
            }

            if (updateTorques)
            {
               jointTorque = revoluteJoint.getTau();
               pinJoint.setTau(jointTorque);
            }

         }
      }

      if (updateRootJoints)
      {
         Collection<? extends FloatingJoint> floatingJoints = scsToInverseDynamicsJointMap.getFloatingJoints(); //floatingToSixDofToJointMap.keySet();
         for (FloatingJoint floatingJoint : floatingJoints)
         {
            FloatingJointBasics sixDoFJoint = scsToInverseDynamicsJointMap.getInverseDynamicsSixDoFJoint(floatingJoint); //floatingToSixDofToJointMap.get(floatingJoint);
            RigidBodyTransform rotationAndTranslation = new RigidBodyTransform();
            if (updatePositions)
            {
               sixDoFJoint.getJointConfiguration(rotationAndTranslation);
               floatingJoint.setRotationAndTranslation(rotationAndTranslation);
            }

            ReferenceFrame pelvisFrame = sixDoFJoint.getFrameAfterJoint();

            if (updateVelocities)
            {
               linearVelocity.setIncludingFrame(pelvisFrame, sixDoFJoint.getJointTwist().getLinearPart());
               linearVelocity.changeFrame(ReferenceFrame.getWorldFrame());
               floatingJoint.setVelocity(linearVelocity);
               floatingJoint.setAngularVelocityInBody(sixDoFJoint.getJointTwist().getAngularPart());
            }

            if (updateAccelerations)
            {
               spatialAccelerationVector.setIncludingFrame(sixDoFJoint.getJointAcceleration());
               originAcceleration.setIncludingFrame(spatialAccelerationVector.getLinearPart());
               originAcceleration.changeFrame(ReferenceFrame.getWorldFrame());
               floatingJoint.setAcceleration(originAcceleration);
               angularAcceleration.setIncludingFrame(spatialAccelerationVector.getAngularPart());
               floatingJoint.setAngularAccelerationInBody(angularAcceleration);
            }
         }
      }
   }
}
