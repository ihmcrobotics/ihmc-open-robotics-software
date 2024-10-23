package us.ihmc.commonWalkingControlModules.virtualModelControl;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.PlanarJoint;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.simulationconstructionset.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;

public class SCSRobotFromInverseDynamicsRobotModel extends Robot
{
   private final FloatingJointBasics idFloatingJoint;
   private final FloatingSCSJoint scsFloatingJoint;

   private final LinkedHashMap<OneDoFJointBasics, OneDegreeOfFreedomJoint> idToSCSJointMap = new LinkedHashMap<OneDoFJointBasics, OneDegreeOfFreedomJoint>();
   private final LinkedHashMap<OneDegreeOfFreedomJoint, OneDoFJointBasics> scsToIDJointMap = new LinkedHashMap<OneDegreeOfFreedomJoint, OneDoFJointBasics>();

   private final ArrayList<OneDoFJointBasics> allIDOneDoFJoints;
   private final ArrayList<OneDegreeOfFreedomJoint> allSCSOneDoFJoints;

   // Temporary variables
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();
   private final FrameVector3D linearVelocity = new FrameVector3D();
   private final FrameVector3D angularVelocity = new FrameVector3D();
   private final Twist rootJointTwist = new Twist();

   public SCSRobotFromInverseDynamicsRobotModel(String name, JointBasics rootJoint)
   {
      super(name);

      Joint scsRootJoint = RobotTools.addSCSJointUsingIDJoint(rootJoint, this, true);
      this.addRootJoint(scsRootJoint);

      ArrayList<JointBasics> idChildJoints = new ArrayList<JointBasics>();
      idChildJoints.addAll(rootJoint.getSuccessor().getChildrenJoints());

      while (!idChildJoints.isEmpty())
      {
         JointBasics currentIDJoint = idChildJoints.remove(0);
         RobotTools.addSCSJointUsingIDJoint(currentIDJoint, this, false);
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

         idToSCSJointMap.put((OneDoFJointBasics) rootJoint, (OneDegreeOfFreedomJoint) scsRootJoint);
         scsToIDJointMap.put((OneDegreeOfFreedomJoint) scsRootJoint, (OneDoFJointBasics) rootJoint);
      }
      else
      {
         throw new RuntimeException("Not implemented yet for joint of the type: " + scsRootJoint.getClass().getSimpleName());
      }

      allSCSOneDoFJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      getAllOneDegreeOfFreedomJoints(allSCSOneDoFJoints);
      allIDOneDoFJoints = new ArrayList<OneDoFJointBasics>(Arrays.asList(MultiBodySystemTools.filterJoints(rootJoint.subtreeArray(), OneDoFJointBasics.class)));

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
         OneDoFJointBasics idJoint = allIDOneDoFJoints.get(i);

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
         OneDoFJointBasics idJoint = scsToIDJointMap.get(scsJoint);
         scsJoint.setQ(idJoint.getQ());
      }
   }

   public void updateJointVelocities_ID_to_SCS()
   {
      if (scsFloatingJoint != null)
      {
         ReferenceFrame rootBodyFrame = idFloatingJoint.getFrameAfterJoint();
         rootJointTwist.setIncludingFrame(idFloatingJoint.getJointTwist());
         linearVelocity.setIncludingFrame(rootJointTwist.getLinearPart());
         angularVelocity.setIncludingFrame(rootJointTwist.getAngularPart());
         linearVelocity.changeFrame(ReferenceFrame.getWorldFrame());
         angularVelocity.changeFrame(rootBodyFrame);
         scsFloatingJoint.setVelocity(linearVelocity);
         scsFloatingJoint.setAngularVelocityInBody(angularVelocity);
      }

      for (OneDegreeOfFreedomJoint scsJoint : allSCSOneDoFJoints)
      {
         OneDoFJointBasics idJoint = scsToIDJointMap.get(scsJoint);
         scsJoint.setQd(idJoint.getQd());
      }
   }

   public void updateJointPositions_SCS_to_ID()
   {
      if (scsFloatingJoint != null)
      {
         scsFloatingJoint.getTransformToWorld(transformToWorld);
         transformToWorld.getRotation().normalize();
         idFloatingJoint.setJointConfiguration(transformToWorld);
      }

      for (OneDegreeOfFreedomJoint scsJoint : allSCSOneDoFJoints)
      {
         OneDoFJointBasics idJoint = scsToIDJointMap.get(scsJoint);
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
         rootJointTwist.setIncludingFrame(rootBodyFrame, elevatorFrame, rootBodyFrame, angularVelocity, linearVelocity);
         idFloatingJoint.setJointTwist(rootJointTwist);
      }

      for (OneDegreeOfFreedomJoint scsJoint : allSCSOneDoFJoints)
      {
         OneDoFJointBasics idJoint = scsToIDJointMap.get(scsJoint);
         idJoint.setQd(scsJoint.getQDYoVariable().getDoubleValue());
      }
   }

   public void updateJointTorques_ID_to_SCS()
   {
      for (OneDegreeOfFreedomJoint scsJoint : allSCSOneDoFJoints)
      {
         OneDoFJointBasics idJoint = scsToIDJointMap.get(scsJoint);
         scsJoint.setTau(idJoint.getTau());
      }
   }

   public void packIdJoints(JointBasics[] idJoints)
   {
      int jointIndx = 0;
      if (idFloatingJoint != null)
      {
         idJoints[0] = idFloatingJoint;
         jointIndx++;
      }

      for (int i = 0; i < allIDOneDoFJoints.size(); i++)
      {
         idJoints[jointIndx] = allIDOneDoFJoints.get(i);
         jointIndx++;
      }
   }

   public PinJoint findSCSPinJoint(JointBasics joint)
   {
      return (PinJoint) idToSCSJointMap.get(joint);
   }
}
