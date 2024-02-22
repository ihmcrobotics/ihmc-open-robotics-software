package us.ihmc.exampleSimulations.planarWalker;


import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
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

    public static final double torsoHeight = 0.5;
    public static final double thighLength = 0.5;
    public static final double shinLength = 0.5;

    private final SideDependentList<String> hipPitchNames = new SideDependentList<>(leftHipPitchName, rightHipPitchName);
    private final SideDependentList<String> thighNames = new SideDependentList<>(leftThighName, rightThighName);
    public static final SideDependentList<String> kneeNames = new SideDependentList<>(leftKneeName, rightKneeName);
    private final SideDependentList<String> shinNames = new SideDependentList<>(leftShinName, rightShinName);



    private final PlanarJointDefinition floatingBaseDefinition;
    private final SideDependentList<RevoluteJointDefinition> hipPitchJointDefinitions = new SideDependentList<>();
    private final RigidBodyDefinition torsoBodyDef;

    public BPWPlanarWalkingRobotDefinition()
    {
        setName(getClass().getSimpleName());

        RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
        setRootBodyDefinition(elevator);

        floatingBaseDefinition = new PlanarJointDefinition("floatingBase");
        elevator.addChildJoint(floatingBaseDefinition);

        torsoBodyDef = createTorso();
        floatingBaseDefinition.setSuccessor(torsoBodyDef);

        for(RobotSide robotside : RobotSide.values)
        {
            // Add the hip to the tree
            Vector3D hipPitchOffsetInTorso = new Vector3D(0.0, robotside.negateIfRightSide(0.05), -torsoHeight/2.0);
            RevoluteJointDefinition hipPitchJD = new RevoluteJointDefinition(hipPitchNames.get(robotside), hipPitchOffsetInTorso, Axis3D.Y);

            torsoBodyDef.addChildJoint(hipPitchJD);
            hipPitchJointDefinitions.put(robotside, hipPitchJD);

            // Now add the thigh links
            // Todo - Attached ment from the center of the link which is usually the middle
            RigidBodyDefinition thighLink = createThigh(thighNames.get(robotside));
            hipPitchJD.setSuccessor(thighLink);

            // Now add the knee which is a type of a prismatic joint
            Vector3D kneeOffsetInThigh = new Vector3D(0.0, 0.0, -thighLength/2.0);
            PrismaticJointDefinition kneeJD = new PrismaticJointDefinition(kneeNames.get(robotside), kneeOffsetInThigh, Axis3D.Y );
            thighLink.addChildJoint(kneeJD);

            // Now add the lower leg
            RigidBodyDefinition lowerLeg = createShin(shinNames.get(robotside));
            kneeJD.setSuccessor(lowerLeg);

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
}
