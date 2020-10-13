package us.ihmc.avatar.reachabilityMap.example;

import java.util.EnumMap;
import java.util.Random;

import us.ihmc.avatar.reachabilityMap.example.RobotParameters.RobotArmJointParameters;
import us.ihmc.avatar.reachabilityMap.example.RobotParameters.RobotArmLinkParameters;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;

public class RobotArm extends Robot
{
   private static final RobotArmJointParameters[] allJointNames = RobotArmJointParameters.values();
   private final EnumMap<RobotArmJointParameters, PinJoint> robotArmPinJoints = new EnumMap<>(RobotArmJointParameters.class);
   private final EnumMap<RobotArmJointParameters, RevoluteJoint> robotArmRevoluteJoints = new EnumMap<>(RobotArmJointParameters.class);
   private final EnumMap<RobotArmLinkParameters, RigidBodyBasics> robotArmRigidBodies = new EnumMap<>(RobotArmLinkParameters.class);

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final RigidBodyTransform elevatorFrameTransformToWorld = new RigidBodyTransform(new AxisAngle(), RobotArmJointParameters.getRootJoint().getJointOffset());
   private final ReferenceFrame elevatorFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("elevatorFrame", worldFrame, elevatorFrameTransformToWorld);
   private final RigidBodyBasics elevator = new RigidBody("elevator", elevatorFrame);

   private final RigidBodyTransform transformFromControlFrameToEndEffectorBodyFixedFrame = new RigidBodyTransform(new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), new Vector3D(0.0, 0.0, -0.08));

   private final ReferenceFrame controlFrame;
   private final GeometricJacobian jacobian;

   private final Random random = new Random(464651L);

   public RobotArm()
   {
      super("RobotArm");

      for (RobotArmJointParameters armJoint : allJointNames)
      {
         PinJoint pinJoint = new PinJoint(armJoint.getJointName(false), armJoint.getJointOffset(), this, armJoint.getJointAxis());
         pinJoint.setLimitStops(armJoint.getJointLowerLimit(), armJoint.getJointUpperLimit(), 100.0, 10.0);
         pinJoint.setDynamic(false);
         robotArmPinJoints.put(armJoint, pinJoint);

         RobotArmLinkParameters armLink = armJoint.getAttachedLink();
         Link link = new Link(armLink.getLinkName());
         link.setLinkGraphics(armLink.getLinkGraphics());
         pinJoint.setLink(link);
      }

      RobotArmJointParameters rootJoint = RobotArmJointParameters.getRootJoint();
      addRootJoint(robotArmPinJoints.get(rootJoint));

      for (RobotArmJointParameters armJoint : allJointNames)
      {
         if (armJoint.getChildJoint() != null)
            robotArmPinJoints.get(armJoint).addJoint(robotArmPinJoints.get(armJoint.getChildJoint()));
      }

      
      RigidBodyBasics parentBody = null;
      RevoluteJoint parentJoint = new RevoluteJoint(rootJoint.getJointName(false), elevator, rootJoint.getJointAxis());
      robotArmRevoluteJoints.put(rootJoint, parentJoint);
      
      for (RobotArmJointParameters armJoint : allJointNames)
      {
         if (robotArmRevoluteJoints.get(armJoint) == null)
         {
            parentJoint = new RevoluteJoint(armJoint.getJointName(false), parentBody, armJoint.getJointOffset(), armJoint.getJointAxis());
            robotArmRevoluteJoints.put(armJoint, parentJoint);
         }

         RobotArmLinkParameters attachedLink = armJoint.getAttachedLink();
         parentBody = new RigidBody(attachedLink.getLinkName(), parentJoint, new Matrix3D(), 1.0, new Vector3D());
         robotArmRigidBodies.put(attachedLink, parentBody);
      }

      for (RobotArmJointParameters armJoint : allJointNames)
      {
         RevoluteJoint robotArmRevoluteJoint = robotArmRevoluteJoints.get(armJoint);
         robotArmRevoluteJoint.setJointLimitLower(armJoint.getJointLowerLimit());
         robotArmRevoluteJoint.setJointLimitUpper(armJoint.getJointUpperLimit());
      }

      RigidBodyBasics endEffector = robotArmRigidBodies.get(RobotArmLinkParameters.getEndEffector());
      controlFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("controlFrame", endEffector.getBodyFixedFrame(), transformFromControlFrameToEndEffectorBodyFixedFrame);
      jacobian = new GeometricJacobian(elevator, endEffector, robotArmRigidBodies.get(RobotArmLinkParameters.HAND).getBodyFixedFrame());
   }

   public GeometricJacobian getJacobian()
   {
      return jacobian;
   }

   public ReferenceFrame getControlFrame()
   {
      return controlFrame;
   }

   public RevoluteJoint getJoint(RobotArmJointParameters armJoint)
   {
      return robotArmRevoluteJoints.get(armJoint);
   }

   public void copyRevoluteJointConfigurationToPinJoints()
   {
      for (RobotArmJointParameters armJoint : allJointNames)
      {
         double q = robotArmRevoluteJoints.get(armJoint).getQ();
         robotArmPinJoints.get(armJoint).setQ(q);
      }
   }

   public void generateRandomConfiguration()
   {
      for (RobotArmJointParameters armJoint : allJointNames)
      {
         double q = RandomNumbers.nextDouble(random, armJoint.getJointLowerLimit(), armJoint.getJointUpperLimit());
         robotArmRevoluteJoints.get(armJoint).setQ(q);
      }
   }

   public void updateFramesRecursively()
   {
      elevator.updateFramesRecursively();
   }

   public RigidBodyTransform getElevatorFrameTransformToWorld()
   {
      return elevatorFrameTransformToWorld;
   }
}
