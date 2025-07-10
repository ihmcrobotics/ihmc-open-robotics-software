package us.ihmc.avatar.reachabilityMap.AlexArm;

import java.util.EnumMap;

import us.ihmc.avatar.reachabilityMap.AlexArm.AlexArmParameters.AlexArmJointParameters;
import us.ihmc.avatar.reachabilityMap.AlexArm.AlexArmParameters.AlexArmLinkParameters;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.definition.robot.KinematicPointDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;

public class AlexArmDefinition extends RobotDefinition
{
    private static final AlexArmJointParameters[] allJointNames = AlexArmJointParameters.values();

    private final EnumMap<AlexArmJointParameters, RevoluteJointDefinition> robotArmRevoluteJoints = new EnumMap<>(AlexArmJointParameters.class);
    private final EnumMap<AlexArmLinkParameters, RigidBodyDefinition> robotArmRigidBodies = new EnumMap<>(AlexArmLinkParameters.class);
    private final RigidBodyTransform transformFromControlFrameToEndEffectorBodyFixedFrame = new RigidBodyTransform(new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0),
            new Vector3D(0.0, 0.0, -0.08));

    public AlexArmDefinition()
    {
        super("AlexArm");

        for (AlexArmJointParameters armJoint : allJointNames)
        {
            RevoluteJointDefinition revoluteJoint = new RevoluteJointDefinition(armJoint.getJointName(false), armJoint.getJointOffset(), armJoint.getJointAxis());
            revoluteJoint.setPositionLimits(armJoint.getJointLowerLimit(), armJoint.getJointUpperLimit());
            robotArmRevoluteJoints.put(armJoint, revoluteJoint);

            AlexArmLinkParameters armLink = armJoint.getAttachedLink();
            RigidBodyDefinition rigidBody = new RigidBodyDefinition(armLink.getLinkName());
            rigidBody.getVisualDefinitions().addAll(armLink.getRigidBodyVisuals());
            revoluteJoint.setSuccessor(rigidBody);
            robotArmRigidBodies.put(armLink, rigidBody);
        }

        RigidBodyDefinition rootBody = new RigidBodyDefinition("rootBody");
        setRootBodyDefinition(rootBody);
        AlexArmJointParameters rootJoint = AlexArmJointParameters.getRootJoint();
        rootBody.addChildJoint(robotArmRevoluteJoints.get(rootJoint));

        for (AlexArmJointParameters armJoint : allJointNames)
        {
            if (armJoint.getChildJoint() != null)
                robotArmRevoluteJoints.get(armJoint).getSuccessor().addChildJoint(robotArmRevoluteJoints.get(armJoint.getChildJoint()));
        }

        RigidBodyDefinition endEffector = robotArmRigidBodies.get(AlexArmLinkParameters.getEndEffector());
        endEffector.getParentJoint()
                .addKinematicPointDefinition(new KinematicPointDefinition("controlFrame", transformFromControlFrameToEndEffectorBodyFixedFrame));
    }
}
