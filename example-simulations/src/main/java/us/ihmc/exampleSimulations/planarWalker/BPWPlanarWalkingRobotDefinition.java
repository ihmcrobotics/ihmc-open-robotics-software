package us.ihmc.exampleSimulations.planarWalker;


import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.definition.geometry.Ellipsoid3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.robot.*;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

import java.awt.*;

public class BPWPlanarWalkingRobotDefinition extends RobotDefinition {


    private static final String torsoName = "torso";
    private static final String leftHipPitchName = "left_hip_pitch";
    private static final String rightHipPitchName = "right_hip_pitch";
    private static final String leftKneeName = "left_knee_pitch";
    private static final String rightKneeName = "right_Knee_pitch";
    private static final String leftThighName = "left_thigh";
    private static final String rightThighName = "right_thigh";

    private static final String leftShinName = "left_Shin";
    private static final String rightShinName = "right_Shin";

    private static final double torsoHeight = 0.5;
    private static final double thighLength = 0.5;
    private static final double shinLength = 0.5;


    private final PlanarJointDefinition floatingBaseDefinition;

    private final RevoluteJointDefinition leftHipJointDefinition;
    private final RevoluteJointDefinition rightHipJointDefinition;

    private final PrismaticJointDefinition leftKneePrismaticJointDefinition;
    private final PrismaticJointDefinition rightKneePrismaticJointDefinition;

    private final RigidBodyDefinition torsoBodyDef;

    private final RigidBodyDefinition leftUpperLeg;
    private final RigidBodyDefinition rightUpperLeg;
    private final RigidBodyDefinition leftLowerLeg;
    private final RigidBodyDefinition rightLowerLeg;

    public BPWPlanarWalkingRobotDefinition()
    {
        setName(getClass().getSimpleName());

        RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
        setRootBodyDefinition(elevator);

        floatingBaseDefinition = new PlanarJointDefinition("floatingBase");
        elevator.addChildJoint(floatingBaseDefinition);

        torsoBodyDef = createTorso();
        floatingBaseDefinition.setSuccessor(torsoBodyDef);

        // Create the Hip Pitch joints and add them to the tree
        Vector3D leftHipPitchOffsetInTorso = new Vector3D(0.0 ,0.05, -torsoHeight/2);
        leftHipJointDefinition = new RevoluteJointDefinition(leftHipPitchName, leftHipPitchOffsetInTorso, Axis3D.Y);

        Vector3D rightHipPitchOffsetInTorso = new Vector3D(0.0 ,-0.05, -torsoHeight/2);
        rightHipJointDefinition = new RevoluteJointDefinition(rightHipPitchName, rightHipPitchOffsetInTorso, Axis3D.Y);

        torsoBodyDef.addChildJoint(leftHipJointDefinition);
        torsoBodyDef.addChildJoint(rightHipJointDefinition);

        // Create the upper leg links and add them to the tree
        // We probably need an offset from the joint attachment to the origin of the link
        leftUpperLeg = createThigh(leftThighName);
        leftHipJointDefinition.setSuccessor(leftUpperLeg);

        rightUpperLeg = createThigh(rightThighName);
        rightHipJointDefinition.setSuccessor(rightUpperLeg);

        // Create the Knee joints and add them to the tree
        Vector3D leftKneeOffsetInThigh = new Vector3D( 0.0, 0.0, -thighLength / 2.0);
        leftKneePrismaticJointDefinition = new PrismaticJointDefinition(leftKneeName, leftKneeOffsetInThigh, Axis3D.Z);
        leftUpperLeg.addChildJoint(leftKneePrismaticJointDefinition);

        Vector3D rightKneeOffsetInThigh = new Vector3D( 0.0, 0.0, -thighLength / 2.0);
        rightKneePrismaticJointDefinition = new PrismaticJointDefinition(rightKneeName, leftKneeOffsetInThigh, Axis3D.Z);
        rightUpperLeg.addChildJoint(rightKneePrismaticJointDefinition);

        // Add the shins
        leftLowerLeg = createShin(leftShinName);
        rightLowerLeg = createShin(rightShinName);

        leftKneePrismaticJointDefinition.setSuccessor((leftLowerLeg));
        rightKneePrismaticJointDefinition.setSuccessor(rightLowerLeg);

    }

    private static RigidBodyDefinition createTorso()
    {
       double torsoMass = 10.0;
       MomentOfInertiaDefinition torsoInertia = new MomentOfInertiaDefinition(0.75,0.75,1.0);

       RigidBodyDefinition torsoBodyDefinition = new RigidBodyDefinition(torsoName);
       torsoBodyDefinition.setMass(torsoMass);
       torsoBodyDefinition.setMomentOfInertia(torsoInertia);

       GeometryDefinition torsoGeometryDefinition = new Ellipsoid3DDefinition(0.15,0.15, torsoHeight / 2.0);
       VisualDefinition torsoVisualDefinition = new VisualDefinition(torsoGeometryDefinition, ColorDefinition.rgb(Color.RED.getRGB()));

       torsoBodyDefinition.addVisualDefinition(torsoVisualDefinition);


       return torsoBodyDefinition;

    }

    private static RigidBodyDefinition createThigh(String name)
    {
        double thighMass = 1.0;
        MomentOfInertiaDefinition thighInertia= new MomentOfInertiaDefinition(0.1,0.1,0.01);

        RigidBodyDefinition thighBodyDefinition = new RigidBodyDefinition(name);
        thighBodyDefinition.setMass(thighMass);
        thighBodyDefinition.setMomentOfInertia(thighInertia);

        GeometryDefinition thighGeometryDefinition = new Ellipsoid3DDefinition(0.05,0.05, thighLength / 2.0);
        VisualDefinition thighVisualDefinition = new VisualDefinition(thighGeometryDefinition, ColorDefinition.rgb(Color.YELLOW.getRGB()));

        thighBodyDefinition.addVisualDefinition(thighVisualDefinition);


        return thighBodyDefinition;
    }

    private static RigidBodyDefinition createShin(String name)
    {
        double shinMass = 1.0;
        MomentOfInertiaDefinition shinInertia= new MomentOfInertiaDefinition(0.1,0.1,0.01);

        RigidBodyDefinition shinBodyDefinition = new RigidBodyDefinition(name);
        shinBodyDefinition.setMass(shinMass);
        shinBodyDefinition.setMomentOfInertia(shinInertia);

        GeometryDefinition shinGeometryDefinition = new Ellipsoid3DDefinition(0.03,0.03, shinLength / 2.0);
        VisualDefinition ShinVizDefinition = new VisualDefinition(shinGeometryDefinition, ColorDefinition.rgb(Color.YELLOW.getRGB()));

        shinBodyDefinition.addVisualDefinition(ShinVizDefinition);


        return shinBodyDefinition;
    }
}
