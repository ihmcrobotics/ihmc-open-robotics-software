package us.ihmc.commonWalkingControlModules.virtualModelControl;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.PlanarJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.robotDescription.Plane;
import us.ihmc.simulationconstructionset.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;

public class RobotTools
{
   public static Joint addSCSJointUsingIDJoint(JointBasics idJoint, Robot scsRobot, boolean isRootJoint)
   {
      Joint scsJoint;
      String jointName = idJoint.getName();
      RigidBodyTransform offsetTransform = new RigidBodyTransform();
      idJoint.getJointOffset(offsetTransform);
      Vector3D offsetVector = new Vector3D();
      offsetVector.set(offsetTransform.getTranslation());

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
         Vector3D axis = new Vector3D(idRevoluteJoint.getJointAxis());
         PinJoint scsRevoluteJoint = new PinJoint(jointName, offsetVector, scsRobot, axis);
         scsJoint = scsRevoluteJoint;
      }
      else
      {
         throw new RuntimeException("Not implemented yet for joint of the type: " + idJoint.getClass().getSimpleName());
      }

      RigidBodyBasics idRigidBody = idJoint.getSuccessor();
      SpatialInertiaBasics idInertia = idRigidBody.getInertia();

      String bodyName = idRigidBody.getName();
      Vector3D comOffset = new Vector3D();
      FramePoint3D centerOfMassOffset = new FramePoint3D(idInertia.getCenterOfMassOffset());
      centerOfMassOffset.changeFrame(idJoint.getFrameAfterJoint());
      comOffset.set(centerOfMassOffset);
      double mass = idInertia.getMass();
      Matrix3D momentOfInertia = new Matrix3D(idInertia.getMomentOfInertia());

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
