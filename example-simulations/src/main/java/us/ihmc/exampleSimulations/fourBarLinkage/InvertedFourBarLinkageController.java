package us.ihmc.exampleSimulations.fourBarLinkage;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotDescription.SliderJointDescription;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class InvertedFourBarLinkageController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getName());
   

   public InvertedFourBarLinkageController(RobotDescription robotDescription, Robot robot)
   {
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void doControl()
   {
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }


   static RigidBodyBasics toInverseDynamicsRobot(RobotDescription description)
   {
      RigidBody rootBody = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      for (JointDescription rootJoint : description.getRootJoints())
         addJointRecursive(rootJoint, rootBody);
      return rootBody;
   }

   static void addJointRecursive(JointDescription jointDescription, RigidBodyBasics parentBody)
   {
      JointBasics joint;
      String name = jointDescription.getName();
      Vector3D jointOffset = new Vector3D();
      jointDescription.getOffsetFromParentJoint(jointOffset);

      if (jointDescription instanceof PinJointDescription)
      {
         Vector3D jointAxis = new Vector3D();
         ((PinJointDescription) jointDescription).getJointAxis(jointAxis);
         joint = new RevoluteJoint(name, parentBody, jointOffset, jointAxis);
      }
      else if (jointDescription instanceof SliderJointDescription)
      {
         Vector3D jointAxis = new Vector3D();
         ((SliderJointDescription) jointDescription).getJointAxis(jointAxis);
         joint = new PrismaticJoint(name, parentBody, jointOffset, jointAxis);
      }
      else if (jointDescription instanceof FloatingJointDescription)
      {
         RigidBodyTransform transformToParent = new RigidBodyTransform();
         transformToParent.getTranslation().set(jointOffset);
         joint = new SixDoFJoint(name, parentBody, transformToParent);
      }
      else
      {
         throw new IllegalStateException("Joint type not handled.");
      }

      LinkDescription linkDescription = jointDescription.getLink();

      String bodyName = linkDescription.getName();
      Matrix3DReadOnly momentOfInertia = linkDescription.getMomentOfInertiaCopy();
      double mass = linkDescription.getMass();
      Tuple3DReadOnly centerOfMassOffset = linkDescription.getCenterOfMassOffset();
      RigidBody successor = new RigidBody(bodyName, joint, momentOfInertia, mass, centerOfMassOffset);

      for (JointDescription childJoint : jointDescription.getChildrenJoints())
         addJointRecursive(childJoint, successor);
   }
}
