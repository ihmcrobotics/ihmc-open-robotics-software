package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.Ellipsoid3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimRevoluteJoint;
import us.ihmc.yoVariables.variable.YoDouble;

import java.awt.*;

public class simplePendulumDefinition extends RobotDefinition {

    public static final String pendulumJointName = "pendulumRevoluteJoint";

    private static final double pendulumLinkLength = 1.0;
    private static final double pendulumLinkRadius = 0.01;
    private static final double pendulumLinkMass = 0.0;
    private static final double revoluteJointDamping = 0.7;

    private static final String linkName = "pendulumLink";
    private static final double pendulumWeight = 10.0;
    private static final double pendulumSphereRadius = 0.1;


    public simplePendulumDefinition()
    {
        setName(getClass().getSimpleName());
        RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
        setRootBodyDefinition(elevator);

        // Add revolute Joint
        Vector3D revJointVector = new Vector3D(0.0, 0.0,2.0);
        RevoluteJointDefinition revoluteJoint = new RevoluteJointDefinition(pendulumJointName, revJointVector, Axis3D.Y);
        revoluteJoint.setDamping(revoluteJointDamping);

        elevator.addChildJoint(revoluteJoint);



        // Add Pendulum Link
        // Define the pendulum Link
        MomentOfInertiaDefinition linkInertia = new MomentOfInertiaDefinition(0.0, pendulumWeight*pendulumLinkLength*pendulumLinkLength,0.00005);
        RigidBodyDefinition linkBodyDefinition = new RigidBodyDefinition(linkName);
//        linkBodyDefinition.
        linkBodyDefinition.setMass(pendulumWeight);
        linkBodyDefinition.setMomentOfInertia(linkInertia);
        linkBodyDefinition.setCenterOfMassOffset(0.0,0.0,-pendulumLinkLength);

        // Adding the visual definitions
        GeometryDefinition pendulumLinkGeometryDefinition = new Cylinder3DDefinition(pendulumLinkLength,pendulumLinkRadius);
        VisualDefinition pendulumLinkVisualDefinition = new VisualDefinition(pendulumLinkGeometryDefinition, ColorDefinition.rgb(Color.RED.getRGB()));
        pendulumLinkVisualDefinition.getOriginPose().appendTranslation(0.0,0.0,-pendulumLinkLength/2.0);
        linkBodyDefinition.addVisualDefinition(pendulumLinkVisualDefinition);


        // Add Mass Visualization at the end
        GeometryDefinition pendulumWeightGeometryDef = new Sphere3DDefinition(pendulumSphereRadius);

        VisualDefinition pendulumWeightVizDef = new VisualDefinition(pendulumWeightGeometryDef, ColorDefinition.rgb(Color.YELLOW.getRGB()));

        pendulumWeightVizDef.getOriginPose().appendTranslation(0.0,0.0,-pendulumLinkLength);

        linkBodyDefinition.addVisualDefinition(pendulumWeightVizDef);

        revoluteJoint.setSuccessor(linkBodyDefinition);



    }

}
