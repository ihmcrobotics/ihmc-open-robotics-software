package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.definition.geometry.Ellipsoid3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.robot.*;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

import java.awt.*;

public class BWCPlanarWalkingRobotDefinition extends RobotDefinition
{
    private static final String torsoName = "torso";
    private static final String leftHipPitchName = "l_hip_pitch";
    private static final String rightHipPitchName = "r_hip_pitch";
    private static final String leftKneeName = "l_knee_pitch";
    private static final String rightKneeName = "r_knee_pitch";
    private static final String leftThighName = "l_thigh";
    private static final String rightThighName = "r_thigh";
    private static final String leftShinName = "l_shin";
    private static final String rightShinName = "r_shin";

    private static final double thighLength = 0.5;
    private static final double torsoHeight = 0.5;
    private static final double shinLength = 0.5;

    private final PlanarJointDefinition floatingBaseDefinition;

    private final RevoluteJointDefinition leftHipPitchJointDefinition;
    private final RevoluteJointDefinition rightHipPitchJointDefinition;

    private final PrismaticJointDefinition leftKneeJointDefinition;
    private final PrismaticJointDefinition rightKneeJointDefinition;

    private final RigidBodyDefinition torsoBodyDefinition;

    private final RigidBodyDefinition leftUpperLeg;
    private final RigidBodyDefinition rightUpperLeg;

    private final RigidBodyDefinition leftLowerLeg;
    private final RigidBodyDefinition rightLowerLeg;

    public BWCPlanarWalkingRobotDefinition()
    {
        setName(getClass().getSimpleName());

        RigidBodyDefinition elevator = new RigidBodyDefinition("elevate");
        setRootBodyDefinition(elevator);

        floatingBaseDefinition = new PlanarJointDefinition("floatingBase");
        elevator.addChildJoint(floatingBaseDefinition);

        torsoBodyDefinition = createTorso();
        floatingBaseDefinition.setSuccessor(torsoBodyDefinition);

        Vector3D leftHipPitchOffsetInTorso = new Vector3D(0.0, 0.05, -torsoHeight / 2.0);
        leftHipPitchJointDefinition = new RevoluteJointDefinition(leftHipPitchName, leftHipPitchOffsetInTorso, Axis3D.Y);

        Vector3D rightHipPitchOffsetInTorso = new Vector3D(0.0, 0.05, -torsoHeight / 2.0);
        rightHipPitchJointDefinition = new RevoluteJointDefinition(rightHipPitchName, rightHipPitchOffsetInTorso, Axis3D.Y);

        torsoBodyDefinition.addChildJoint(leftHipPitchJointDefinition);
        torsoBodyDefinition.addChildJoint(rightHipPitchJointDefinition);

        // Create the upper leg links and add them to the tree
        leftUpperLeg = createLeftThigh(leftThighName);
        leftHipPitchJointDefinition.setSuccessor(leftUpperLeg);

        rightUpperLeg = createLeftThigh(rightThighName);
        rightHipPitchJointDefinition.setSuccessor(rightUpperLeg);

        //TODO we probably need to add an offset from the joint attachment to the origin of the link;

        // Create the knee joints and add them to the tree
        Vector3D leftKneeOffsetInThigh = new Vector3D(0.0, 0.0, -thighLength / 2.0);
        leftKneeJointDefinition = new PrismaticJointDefinition(leftKneeName, leftKneeOffsetInThigh, Axis3D.Z);
        leftUpperLeg.addChildJoint(leftKneeJointDefinition);

        Vector3D rightKneeOffsetInThigh = new Vector3D(0.0, 0.0, -thighLength / 2.0);
        rightKneeJointDefinition = new PrismaticJointDefinition(rightKneeName, rightKneeOffsetInThigh, Axis3D.Z);
        rightUpperLeg.addChildJoint(rightKneeJointDefinition);

        // Create the shin links and add them to the tree
        leftLowerLeg = createShin(leftShinName);
        leftKneeJointDefinition.setSuccessor(leftLowerLeg);
        rightLowerLeg = createShin(rightShinName);
        rightKneeJointDefinition.setSuccessor(rightLowerLeg);
    }


    private RigidBodyDefinition createTorso()
    {
        double torsoMass = 10.0;
        MomentOfInertiaDefinition torsoMomentOfInertia = new MomentOfInertiaDefinition(0.75, 0.75, 1.0);

        RigidBodyDefinition torsoBodyDefinition = new RigidBodyDefinition(torsoName);
        torsoBodyDefinition.setMass(torsoMass);
        torsoBodyDefinition.setMomentOfInertia(torsoMomentOfInertia);

        GeometryDefinition torsoGeometryDefinition = new Ellipsoid3DDefinition(0.15, 0.15, torsoHeight / 2.0);
        VisualDefinition torsoVisualDefinition = new VisualDefinition(torsoGeometryDefinition, ColorDefinition.argb(Color.RED.getRGB()));

        torsoBodyDefinition.addVisualDefinition(torsoVisualDefinition);

        return torsoBodyDefinition;
    }


    private static RigidBodyDefinition createLeftThigh(String name)
    {
        double thighMass = 1.0;
        MomentOfInertiaDefinition thighMomentOfInertia = new MomentOfInertiaDefinition(0.1, 0.1, 0.01);

        RigidBodyDefinition thighDefinition = new RigidBodyDefinition(name);
        thighDefinition.setMass(thighMass);
        thighDefinition.setMomentOfInertia(thighMomentOfInertia);

        GeometryDefinition thighGeometryDefinition = new Ellipsoid3DDefinition(0.35, 0.05, thighLength / 2.0);
        VisualDefinition thighVisualDefinition = new VisualDefinition(thighGeometryDefinition, ColorDefinition.rgb(Color.ORANGE.getRGB()));

        thighDefinition.addVisualDefinition(thighVisualDefinition);

        return thighDefinition;
    }

    private static RigidBodyDefinition createShin(String name)
    {
        double shinMass = 1.0;
        MomentOfInertiaDefinition thighMomentOfInertia = new MomentOfInertiaDefinition(0.1, 0.1, 0.01);

        RigidBodyDefinition shinDefinition = new RigidBodyDefinition(name);
        shinDefinition.setMass(shinMass);
        shinDefinition.setMomentOfInertia(thighMomentOfInertia);

        GeometryDefinition shinGeometryDefinition = new Ellipsoid3DDefinition(0.03, 0.03, shinLength / 2.0);
        VisualDefinition shinVisualDefinition = new VisualDefinition(shinGeometryDefinition, ColorDefinition.argb(Color.YELLOW.getRGB()));

        shinDefinition.addVisualDefinition(shinVisualDefinition);

        return shinDefinition;
    }
}
