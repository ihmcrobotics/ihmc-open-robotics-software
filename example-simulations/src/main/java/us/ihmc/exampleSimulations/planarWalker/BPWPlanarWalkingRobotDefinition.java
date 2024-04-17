package us.ihmc.exampleSimulations.planarWalker;


import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.geometry.Ellipsoid3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.robot.*;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import java.awt.*;

public class BPWPlanarWalkingRobotDefinition extends RobotDefinition {


    public static final String baseJointName = "floatingBase";
    public static final String torsoName = "torso";
    private static final String leftHipPitchName = "left_hip_pitch";
    private static final String rightHipPitchName = "right_hip_pitch";
    private static final String leftHipRollName = "left_hip_roll";
    private static final String rightHipRollName = "right_hip_roll";
    private static final String leftKneeName = "left_knee_pitch";
    private static final String rightKneeName = "right_Knee_pitch";
    private static final String leftThighName = "left_thigh";
    private static final String rightThighName = "right_thigh";

    private static final String leftShinName = "left_Shin";
    private static final String rightShinName = "right_Shin";

    private static final String leftHipLink = "left_hip_link";
    private static final String rightHipLink = "right_hip_link";

    public static final double torsoHeight = 0.5;
    public static final double thighLength = 0.5;
    public static final double shinLength = 0.5;

    public static final SideDependentList<String> hipPitchNames = new SideDependentList<>(leftHipPitchName, rightHipPitchName);
    public static final SideDependentList<String> hipRollNames = new SideDependentList<>(leftHipRollName, rightHipRollName);
    public static final SideDependentList<String> hipLinkNames = new SideDependentList<>(leftHipLink, rightHipLink);
    private final SideDependentList<String> thighNames = new SideDependentList<>(leftThighName, rightThighName);
    public static final SideDependentList<String> kneeNames = new SideDependentList<>(leftKneeName, rightKneeName);
    private final SideDependentList<String> shinNames = new SideDependentList<>(leftShinName, rightShinName);



    private final SixDoFJointDefinition floatingBaseDefinition;
    private final SideDependentList<RevoluteJointDefinition> hipPitchJointDefinitions = new SideDependentList<>();
    private final SideDependentList<RevoluteJointDefinition> hipRollJointDefinitions = new SideDependentList<>();
    private final RigidBodyDefinition torsoBodyDef;

    public BPWPlanarWalkingRobotDefinition()
    {
        setName(getClass().getSimpleName());

        RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
        setRootBodyDefinition(elevator);

        floatingBaseDefinition = new SixDoFJointDefinition(baseJointName);
        elevator.addChildJoint(floatingBaseDefinition);

        torsoBodyDef = createTorso();
        floatingBaseDefinition.setSuccessor(torsoBodyDef);

        KinematicPointDefinition basePoseDefinition = new KinematicPointDefinition("_base");
        floatingBaseDefinition.addKinematicPointDefinition(basePoseDefinition);

        for(RobotSide robotside : RobotSide.values)
        {

            // Add First Hip revolute joint that rotates about X
            Vector3D hipRollOffsetInTorso = new Vector3D(0.0, robotside.negateIfRightSide(0.05), -torsoHeight/2.0);
            RevoluteJointDefinition hipRollJDX = new RevoluteJointDefinition(hipRollNames.get(robotside), hipRollOffsetInTorso, Axis3D.X);
            hipRollJDX.setPositionLimits(-Math.PI/2.0, Math.PI/2.0 );

            torsoBodyDef.addChildJoint(hipRollJDX);
            hipRollJointDefinitions.put(robotside, hipRollJDX);

            // Add a massless link before adding the second revolute joint that rotates about Y
            RigidBodyDefinition hipMasslessLink = createMasslessHipLink(hipLinkNames.get(robotside));
            hipRollJDX.setSuccessor(hipMasslessLink);

            // Add the second hip revolute joint to the tree which rotates about Y and allows roll
            Vector3D hipPitchOffset = new Vector3D(0.0, 0.0, 0.0);
            RevoluteJointDefinition hipPitchJDY = new RevoluteJointDefinition(hipPitchNames.get(robotside), hipPitchOffset, Axis3D.Y);
            hipPitchJDY.setPositionLimits(-Math.PI/2.0, Math.PI/2.0 );

            hipMasslessLink.addChildJoint(hipPitchJDY);
            hipPitchJointDefinitions.put(robotside, hipPitchJDY);

             // Now add the thigh links
            // Todo - Attached ment from the center of the link which is usually the middle
            RigidBodyDefinition thighLink = createThigh(thighNames.get(robotside));
            hipPitchJDY.setSuccessor(thighLink);

            KinematicPointDefinition hipPoseDefinition = new KinematicPointDefinition(robotside.getLowerCaseName() + "_hip");
            hipPitchJDY.addKinematicPointDefinition(hipPoseDefinition);

            // Now add the knee which is a type of a prismatic joint
            Vector3D kneeOffsetInThigh = new Vector3D(0.0, 0.0, -thighLength/2.0);
            PrismaticJointDefinition kneeJD = new PrismaticJointDefinition(kneeNames.get(robotside), kneeOffsetInThigh, Axis3D.Z );
            kneeJD.setPositionLimits(-shinLength/2.0, shinLength/2.0 );

//            RevoluteJointDefinition kneeJD = new RevoluteJointDefinition(kneeNames.get(robotside), kneeOffsetInThigh, Axis3D.Y);
//            kneeJD.setPositionLimits(-Math.PI/2.0, Math.PI/2.0 );

            thighLink.addChildJoint(kneeJD);

            // Now add the lower leg
            RigidBodyDefinition lowerLeg = createShin(shinNames.get(robotside));
            kneeJD.setSuccessor(lowerLeg);

            KinematicPointDefinition kneePoseDefinition = new KinematicPointDefinition(robotside.getLowerCaseName() + "_knee");
            kneeJD.addKinematicPointDefinition(kneePoseDefinition);

            // Create the contact points for the feet
            GroundContactPointDefinition footContactPoint = new GroundContactPointDefinition(robotside.getShortLowerCaseName() + "_gc_point", new Vector3D(0.0,0.0, -shinLength/2.0));
            kneeJD.addGroundContactPointDefinition(footContactPoint);

        }

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

    private static RigidBodyDefinition createMasslessHipLink(String name)
    {
        double hipLinkMass = 0.0;

        MomentOfInertiaDefinition hipLinkInertia = new MomentOfInertiaDefinition(0.0,0.0,0.0);

        RigidBodyDefinition hipLinkDefinition = new RigidBodyDefinition(name);
        hipLinkDefinition.setMass(hipLinkMass);
        hipLinkDefinition.setMomentOfInertia(hipLinkInertia);

        return hipLinkDefinition;


    }
}
