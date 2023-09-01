package us.ihmc.avatar.reachabilityMap.example;

import java.util.EnumMap;

import us.ihmc.avatar.reachabilityMap.example.RobotParameters.RobotArmJointParameters;
import us.ihmc.avatar.reachabilityMap.example.RobotParameters.RobotArmLinkParameters;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.definition.robot.KinematicPointDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;

public class RobotArmDefinition extends RobotDefinition
{
   private static final RobotArmJointParameters[] allJointNames = RobotArmJointParameters.values();

   private final EnumMap<RobotArmJointParameters, RevoluteJointDefinition> robotArmRevoluteJoints = new EnumMap<>(RobotArmJointParameters.class);
   private final EnumMap<RobotArmLinkParameters, RigidBodyDefinition> robotArmRigidBodies = new EnumMap<>(RobotArmLinkParameters.class);
   private final RigidBodyTransform transformFromControlFrameToEndEffectorBodyFixedFrame = new RigidBodyTransform(new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0),
                                                                                                                  new Vector3D(0.0, 0.0, -0.08));

   public RobotArmDefinition()
   {
      super("RobotArm");

      for (RobotArmJointParameters armJoint : allJointNames)
      {
         RevoluteJointDefinition revoluteJoint = new RevoluteJointDefinition(armJoint.getJointName(false), armJoint.getJointOffset(), armJoint.getJointAxis());
         revoluteJoint.setPositionLimits(armJoint.getJointLowerLimit(), armJoint.getJointUpperLimit());
         robotArmRevoluteJoints.put(armJoint, revoluteJoint);

         RobotArmLinkParameters armLink = armJoint.getAttachedLink();
         RigidBodyDefinition rigidBody = new RigidBodyDefinition(armLink.getLinkName());
         rigidBody.getVisualDefinitions().addAll(armLink.getRigidBodyVisuals());
         revoluteJoint.setSuccessor(rigidBody);
         robotArmRigidBodies.put(armLink, rigidBody);
      }

      RigidBodyDefinition rootBody = new RigidBodyDefinition("rootBody");
      setRootBodyDefinition(rootBody);
      RobotArmJointParameters rootJoint = RobotArmJointParameters.getRootJoint();
      rootBody.addChildJoint(robotArmRevoluteJoints.get(rootJoint));

      for (RobotArmJointParameters armJoint : allJointNames)
      {
         if (armJoint.getChildJoint() != null)
            robotArmRevoluteJoints.get(armJoint).getSuccessor().addChildJoint(robotArmRevoluteJoints.get(armJoint.getChildJoint()));
      }

      RigidBodyDefinition endEffector = robotArmRigidBodies.get(RobotArmLinkParameters.getEndEffector());
      endEffector.getParentJoint()
                 .addKinematicPointDefinition(new KinematicPointDefinition("controlFrame", transformFromControlFrameToEndEffectorBodyFixedFrame));
   }
}
