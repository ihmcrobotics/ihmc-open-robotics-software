package us.ihmc.simulationconstructionset;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.Plane;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.PlanarJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.RigidBodyInertia;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.Twist;

public class RobotTools
{
   public static class SCSRobotFromInverseDynamicsRobotModel extends Robot
   {
      private final FloatingInverseDynamicsJoint idFloatingJoint;
      private final FloatingSCSJoint scsFloatingJoint;

      private final LinkedHashMap<OneDoFJoint, OneDegreeOfFreedomJoint> idToSCSJointMap = new LinkedHashMap<OneDoFJoint, OneDegreeOfFreedomJoint>();
      private final LinkedHashMap<OneDegreeOfFreedomJoint, OneDoFJoint> scsToIDJointMap = new LinkedHashMap<OneDegreeOfFreedomJoint, OneDoFJoint>();

      private final ArrayList<OneDoFJoint> allIDOneDoFJoints;
      private final ArrayList<OneDegreeOfFreedomJoint> allSCSOneDoFJoints;

      // Temporary variables
      private final RigidBodyTransform transformToWorld = new RigidBodyTransform();
      private final FrameVector linearVelocity = new FrameVector();
      private final FrameVector angularVelocity = new FrameVector();
      private final Twist rootJointTwist = new Twist();

      public SCSRobotFromInverseDynamicsRobotModel(String name, InverseDynamicsJoint rootJoint)
      {
         super(name);

         Joint scsRootJoint = addSCSJointUsingIDJoint(rootJoint, this, true);
         this.addRootJoint(scsRootJoint);

         ArrayList<InverseDynamicsJoint> idChildJoints = new ArrayList<InverseDynamicsJoint>();
         idChildJoints.addAll(rootJoint.getSuccessor().getChildrenJoints());

         while (!idChildJoints.isEmpty())
         {
            InverseDynamicsJoint currentIDJoint = idChildJoints.remove(0);
            addSCSJointUsingIDJoint(currentIDJoint, this, false);
            idChildJoints.addAll(currentIDJoint.getSuccessor().getChildrenJoints());
         }

         if (scsRootJoint instanceof FloatingJoint)
         {
            scsFloatingJoint = (FloatingJoint) scsRootJoint;
            idFloatingJoint = (SixDoFJoint) rootJoint;
         }
         else if (scsRootJoint instanceof FloatingPlanarJoint)
         {
            scsFloatingJoint = (FloatingPlanarJoint) scsRootJoint;
            idFloatingJoint = (PlanarJoint) rootJoint;
         }
         else if (scsRootJoint instanceof OneDegreeOfFreedomJoint)
         {
            scsFloatingJoint = null;
            idFloatingJoint = null;

            idToSCSJointMap.put((OneDoFJoint) rootJoint, (OneDegreeOfFreedomJoint) scsRootJoint);
            scsToIDJointMap.put((OneDegreeOfFreedomJoint) scsRootJoint, (OneDoFJoint) rootJoint);
         }
         else
         {
            throw new RuntimeException("Not implemented yet for joint of the type: " + scsRootJoint.getClass().getSimpleName());
         }

         allSCSOneDoFJoints = new ArrayList<OneDegreeOfFreedomJoint>();
         getAllOneDegreeOfFreedomJoints(allSCSOneDoFJoints);
         allIDOneDoFJoints = new ArrayList<OneDoFJoint>(Arrays.asList(ScrewTools.filterJoints(ScrewTools.computeSubtreeJoints(rootJoint.getPredecessor()),
               OneDoFJoint.class)));

         if (allIDOneDoFJoints.size() != allSCSOneDoFJoints.size())
            throw new RuntimeException("Should not get there...");

         LinkedHashMap<String, OneDegreeOfFreedomJoint> scsJointsByName = new LinkedHashMap<String, OneDegreeOfFreedomJoint>();

         for (int i = 0; i < allSCSOneDoFJoints.size(); i++)
         {
            OneDegreeOfFreedomJoint scsJoint = allSCSOneDoFJoints.get(i);
            scsJointsByName.put(scsJoint.getName(), scsJoint);
         }

         for (int i = 0; i < allIDOneDoFJoints.size(); i++)
         {
            OneDoFJoint idJoint = allIDOneDoFJoints.get(i);

            idToSCSJointMap.put(idJoint, scsJointsByName.get(idJoint.getName()));
            scsToIDJointMap.put(scsJointsByName.get(idJoint.getName()), idJoint);
         }
      }

      public void updateJointPositions_ID_to_SCS()
      {
         if (scsFloatingJoint != null)
         {
            idFloatingJoint.getFrameAfterJoint().getTransformToDesiredFrame(transformToWorld, ReferenceFrame.getWorldFrame());
            scsFloatingJoint.setRotationAndTranslation(transformToWorld);
         }

         for (OneDegreeOfFreedomJoint scsJoint : allSCSOneDoFJoints)
         {
            OneDoFJoint idJoint = scsToIDJointMap.get(scsJoint);
            scsJoint.setQ(idJoint.getQ());
         }
      }

      public void updateJointVelocities_ID_to_SCS()
      {
         if (scsFloatingJoint != null)
         {
            ReferenceFrame rootBodyFrame = idFloatingJoint.getFrameAfterJoint();
            idFloatingJoint.getJointTwist(rootJointTwist);
            rootJointTwist.getLinearPart(linearVelocity);
            rootJointTwist.getAngularPart(angularVelocity);
            linearVelocity.changeFrame(ReferenceFrame.getWorldFrame());
            angularVelocity.changeFrame(rootBodyFrame);
            scsFloatingJoint.setVelocity(linearVelocity.getVector());
            scsFloatingJoint.setAngularVelocityInBody(angularVelocity.getVector());
         }

         for (OneDegreeOfFreedomJoint scsJoint : allSCSOneDoFJoints)
         {
            OneDoFJoint idJoint = scsToIDJointMap.get(scsJoint);
            scsJoint.setQd(idJoint.getQd());
         }
      }

      public void updateJointPositions_SCS_to_ID()
      {
         if (scsFloatingJoint != null)
         {
            scsFloatingJoint.getTransformToWorld(transformToWorld);
            transformToWorld.normalizeRotationPart();
            idFloatingJoint.setPositionAndRotation(transformToWorld);
         }

         for (OneDegreeOfFreedomJoint scsJoint : allSCSOneDoFJoints)
         {
            OneDoFJoint idJoint = scsToIDJointMap.get(scsJoint);
            idJoint.setQ(scsJoint.getQYoVariable().getDoubleValue());
         }
      }

      public void updateJointVelocities_SCS_to_ID()
      {
         if (scsFloatingJoint != null)
         {
            ReferenceFrame elevatorFrame = idFloatingJoint.getFrameBeforeJoint();
            ReferenceFrame rootBodyFrame = idFloatingJoint.getFrameAfterJoint();
            scsFloatingJoint.getVelocity(linearVelocity);
            linearVelocity.changeFrame(rootBodyFrame);
            scsFloatingJoint.getAngularVelocity(angularVelocity, rootBodyFrame);
            rootJointTwist.set(rootBodyFrame, elevatorFrame, rootBodyFrame, linearVelocity.getVector(), angularVelocity.getVector());
            idFloatingJoint.setJointTwist(rootJointTwist);
         }

         for (OneDegreeOfFreedomJoint scsJoint : allSCSOneDoFJoints)
         {
            OneDoFJoint idJoint = scsToIDJointMap.get(scsJoint);
            idJoint.setQd(scsJoint.getQDYoVariable().getDoubleValue());
         }
      }

      public void updateJointTorques_ID_to_SCS()
      {
         for (OneDegreeOfFreedomJoint scsJoint : allSCSOneDoFJoints)
         {
            OneDoFJoint idJoint = scsToIDJointMap.get(scsJoint);
            scsJoint.setTau(idJoint.getTau());
         }
      }
      
      public void packIdJoints(InverseDynamicsJoint[] idJoints)
      {
         int jointIndx = 0;
         if(idFloatingJoint != null)
         {
            idJoints[0] = idFloatingJoint;
            jointIndx++;
            
         }
         
         for(int i = 0; i<allIDOneDoFJoints.size();i++)
         {
            idJoints[jointIndx] = allIDOneDoFJoints.get(i);
            jointIndx++;
         }
         
      }
           
      public PinJoint findSCSPinJoint(InverseDynamicsJoint joint)
      {
         return (PinJoint) idToSCSJointMap.get(joint);
      }
   }

   public static Joint addSCSJointUsingIDJoint(InverseDynamicsJoint idJoint, Robot scsRobot, boolean isRootJoint)
   {
      Joint scsJoint;
      String jointName = idJoint.getName();
      RigidBodyTransform offsetTransform = idJoint.getOffsetTransform3D();
      Vector3D offsetVector = new Vector3D();
      offsetTransform.getTranslation(offsetVector);

      if (idJoint instanceof SixDoFJoint)
      {
         if (!isRootJoint)
            throw new RuntimeException("Should not have a SixDoFJoint in the middle of the robot.");

         FloatingJoint scsSixDoFJoint = new FloatingJoint(jointName, offsetVector, scsRobot);
         scsJoint = scsSixDoFJoint;
      }
      else if (idJoint instanceof PlanarJoint)
      {
         if (!isRootJoint)
            throw new RuntimeException("Should not have a PlanarJoint in the middle of the robot.");

         FloatingPlanarJoint scsPlanarJoint = new FloatingPlanarJoint(jointName, offsetVector, scsRobot, Plane.XZ);
         scsJoint = scsPlanarJoint;
      }
      else if (idJoint instanceof RevoluteJoint)
      {
         RevoluteJoint idRevoluteJoint = (RevoluteJoint) idJoint;
         Vector3D axis = new Vector3D();
         idRevoluteJoint.getJointAxis().get(axis);
         PinJoint scsRevoluteJoint = new PinJoint(jointName, offsetVector, scsRobot, axis);
         scsJoint = scsRevoluteJoint;
      }
      else
      {
         throw new RuntimeException("Not implemented yet for joint of the type: " + idJoint.getClass().getSimpleName());
      }

      RigidBody idRigidBody = idJoint.getSuccessor();
      RigidBodyInertia idInertia = idRigidBody.getInertia();

      String bodyName = idRigidBody.getName();
      Vector3D comOffset = new Vector3D();
      FramePoint centerOfMassOffset = idInertia.getCenterOfMassOffset();
      centerOfMassOffset.changeFrame(idJoint.getFrameAfterJoint());
      centerOfMassOffset.get(comOffset);
      double mass = idInertia.getMass();
      Matrix3D momentOfInertia = idInertia.getMassMomentOfInertiaPartCopy();

      Link scsRigidBody = new Link(bodyName);
      scsRigidBody.setComOffset(comOffset);
      scsRigidBody.setMass(mass);
      scsRigidBody.setMomentOfInertia(momentOfInertia);

      scsJoint.setLink(scsRigidBody);

      if (!isRootJoint)
      {
         ArrayList<OneDegreeOfFreedomJoint> allSCSRobotOneDoFJoints = new ArrayList<OneDegreeOfFreedomJoint>();
         scsRobot.getAllOneDegreeOfFreedomJoints(allSCSRobotOneDoFJoints);

         ArrayList<Joint> allSCSJoints = new ArrayList<Joint>();
         allSCSJoints.addAll(allSCSRobotOneDoFJoints);
         allSCSJoints.addAll(scsRobot.getRootJoints());

         Joint parentSCSJoint = null;

         for (Joint currentSCSJoint : allSCSJoints)
         {
            if (currentSCSJoint.getName().equals(idJoint.getPredecessor().getParentJoint().getName()))
            {
               parentSCSJoint = currentSCSJoint;
               break;
            }
         }

         if (parentSCSJoint == null)
            throw new RuntimeException("Did not find parent joint.");

         parentSCSJoint.addJoint(scsJoint);
      }
      return scsJoint;
   }
}
