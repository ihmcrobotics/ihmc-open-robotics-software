package us.ihmc.simulationToolkit.physicsEngine;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.lists.RingBuffer;
import us.ihmc.robotics.physics.ExternalWrenchReader;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Robot;

public class SCSRobotExternalWrenchReader implements ExternalWrenchReader
{
   private int movingAverageLength = 3;
   private final List<ExtendedContactPointBundle> contactPointBundles = new ArrayList<>();

   public SCSRobotExternalWrenchReader()
   {
   }

   public void addRobot(RigidBodyReadOnly rootBody, Robot scsRobot)
   {
      List<List<GroundContactPoint>> allGroundContactPointsGroupedByJoint = scsRobot.getAllGroundContactPointsGroupedByJoint();
      JointReadOnly[] allJoint = MultiBodySystemTools.collectSubtreeJoints(rootBody);

      for (List<GroundContactPoint> groundContactPointBundle : allGroundContactPointsGroupedByJoint)
      {
         if (groundContactPointBundle.isEmpty())
            continue;

         Joint scsJoint = groundContactPointBundle.get(0).getParentJoint();
         JointReadOnly joint = Stream.of(allJoint).filter(candidate -> candidate.getName().equals(scsJoint.getName())).findFirst().get();

         contactPointBundles.add(new ExtendedContactPointBundle(joint, groundContactPointBundle));
      }
   }

   public void initialize()
   {
      contactPointBundles.forEach(ExtendedContactPointBundle::initialize);
   }

   @Override
   public void readExternalWrench(RigidBodyReadOnly rigidBody, WrenchReadOnly externalWrench)
   {
      contactPointBundles.forEach(bundle -> bundle.readExternalWrench(rigidBody, externalWrench));
   }

   public void updateSCSGroundContactPoints()
   {
      contactPointBundles.forEach(ExtendedContactPointBundle::updateSCSGroundContactPointBundle);
   }

   private class ExtendedContactPointBundle
   {
      private final RigidBodyReadOnly rigidBody;
      private final List<ExtendedContactPoint> extendedContactPoints;

      public ExtendedContactPointBundle(JointReadOnly joint, List<GroundContactPoint> scsGroundContactPointBundle)
      {
         Joint scsParentJoint = scsGroundContactPointBundle.get(0).getParentJoint();

         if (!scsGroundContactPointBundle.stream().allMatch(scsGC -> scsGC.getParentJoint() == scsParentJoint))
            throw new IllegalArgumentException("The bundle of contact points are not attached to the same joint.");

         rigidBody = joint.getSuccessor();
         extendedContactPoints = scsGroundContactPointBundle.stream().map(scsGC -> new ExtendedContactPoint(joint, scsGC)).collect(Collectors.toList());
      }

      public void initialize()
      {
         extendedContactPoints.forEach(ExtendedContactPoint::initialize);
      }

      private final Wrench scaledWrench = new Wrench();

      public void readExternalWrench(RigidBodyReadOnly rigidBody, WrenchReadOnly externalWrench)
      {
         if (rigidBody != this.rigidBody)
            return;

         scaledWrench.setIncludingFrame(externalWrench);
         scaledWrench.scale(1.0 / extendedContactPoints.size());
         extendedContactPoints.forEach(contactPoint -> contactPoint.readExternalWrench(rigidBody, scaledWrench));
      }

      public void updateSCSGroundContactPointBundle()
      {
         extendedContactPoints.forEach(ExtendedContactPoint::updateSCSGroundContactPoint);
      }
   }

   private class ExtendedContactPoint
   {
      private final RigidBodyReadOnly rigidBody;
      private final GroundContactPoint groundContactPoint;
      private final FramePoint3D position = new FramePoint3D();
      private final RingBuffer<Wrench> wrenchBuffer;
      private final Wrench averageWrench = new Wrench();
      private final FrameVector3D force = new FrameVector3D();
      private final FrameVector3D moment = new FrameVector3D();

      public ExtendedContactPoint(JointReadOnly joint, GroundContactPoint groundContactPoint)
      {
         this.groundContactPoint = groundContactPoint;

         rigidBody = joint.getSuccessor();
         position.setIncludingFrame(joint.getFrameAfterJoint(), groundContactPoint.getOffsetCopy());
         position.changeFrame(rigidBody.getBodyFixedFrame());

         wrenchBuffer = new RingBuffer<Wrench>(movingAverageLength, () -> new Wrench(rigidBody.getBodyFixedFrame(), rigidBody.getBodyFixedFrame()));
      }

      private Wrench currentWrench;

      public void initialize()
      {
         if (movingAverageLength != wrenchBuffer.capacity())
            wrenchBuffer.changeCapacity(movingAverageLength);

         currentWrench = wrenchBuffer.add();
         currentWrench.setToZero(rigidBody.getBodyFixedFrame(), rigidBody.getBodyFixedFrame());
         averageWrench.setToZero(rigidBody.getBodyFixedFrame(), rigidBody.getBodyFixedFrame());
      }

      public void readExternalWrench(RigidBodyReadOnly rigidBody, WrenchReadOnly externalWrench)
      {
         currentWrench.add(externalWrench);
      }

      public void updateSCSGroundContactPoint()
      {
         wrenchBuffer.forEach(wrench -> averageWrench.add(wrench));
         averageWrench.scale(1.0 / wrenchBuffer.size());

         force.setMatchingFrame(averageWrench.getLinearPart());
         averageWrench.getAngularPartAt(position, moment);
         moment.changeFrame(ReferenceFrame.getWorldFrame());

         groundContactPoint.setForce(force);
         groundContactPoint.setMoment(moment.getX(), moment.getY(), moment.getZ());
      }
   }
}
