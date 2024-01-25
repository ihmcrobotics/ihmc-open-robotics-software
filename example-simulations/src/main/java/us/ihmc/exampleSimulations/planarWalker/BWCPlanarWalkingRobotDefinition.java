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
   public static final String torsoName = "torso";
   public static final String leftThighName = "leftThigh";
   public static final String rightThighName = "rightThigh";
   public static final String leftShinName = "leftShin";
   public static final String rightShinName = "rightShin";

   public static final String leftHipPitchName = "leftHipPitch";
   public static final String rightHipPitchName = "rightHipPitch";
   public static final String leftKneeName = "leftKnee";
   public static final String rightKneeName = "rightKnee";

   private static final double torsoLength = 0.5;
   private static final double thighLength = 0.5;
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

      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
      setRootBodyDefinition(elevator);
      floatingBaseDefinition = new PlanarJointDefinition("floatingBase");
      elevator.addChildJoint(floatingBaseDefinition);

      torsoBodyDefinition = createTorso();
      floatingBaseDefinition.setSuccessor(torsoBodyDefinition);

      // Create the hip pitch joints and add them to the tree
      Vector3D leftHipPitchOffsetTorso = new Vector3D(0.0, 0.05, -0.2);
      leftHipPitchJointDefinition = new RevoluteJointDefinition(leftHipPitchName, leftHipPitchOffsetTorso, Axis3D.Y);

      Vector3D rightHipPitchOffsetTorso = new Vector3D(0.0, -0.05, -0.2);
      rightHipPitchJointDefinition = new RevoluteJointDefinition(rightHipPitchName, rightHipPitchOffsetTorso, Axis3D.Y);

      torsoBodyDefinition.addChildJoint(leftHipPitchJointDefinition);
      torsoBodyDefinition.addChildJoint(rightHipPitchJointDefinition);

      // Create the upper leg links and add them to the tree
      // FIXME: we probably need to add an offset so the link isn't placed at the origin
      leftUpperLeg = createThigh(leftThighName);
      leftHipPitchJointDefinition.setSuccessor(leftUpperLeg);

      rightUpperLeg = createThigh(rightThighName);
      rightHipPitchJointDefinition.setSuccessor(rightUpperLeg);

      // Create the knee joints and add them to the tree
      Vector3D leftKneeOffsetInThigh = new Vector3D(0.0, 0.0, -thighLength/2.0);
      leftKneeJointDefinition = new PrismaticJointDefinition(leftKneeName, leftKneeOffsetInThigh, Axis3D.Z);
      leftUpperLeg.addChildJoint(leftKneeJointDefinition);

      Vector3D rightKneeOffsetInThigh = new Vector3D(0.0, 0.0, -thighLength/2.0);
      rightKneeJointDefinition = new PrismaticJointDefinition(rightKneeName, rightKneeOffsetInThigh, Axis3D.Z);
      rightUpperLeg.addChildJoint(rightKneeJointDefinition);

      // Create the lower leg links and add them to the tree
      // FIXME: we probably need to add an offset so the link isn't placed at the origin
      leftLowerLeg = createShin(leftShinName);
      leftKneeJointDefinition.setSuccessor(leftLowerLeg);

      rightLowerLeg = createShin(rightShinName);
      rightKneeJointDefinition.setSuccessor(rightLowerLeg);

      //TODO: add collisions shapes
   }

   private RigidBodyDefinition createTorso()
   {
      RigidBodyDefinition torsoBodyDefinition = new RigidBodyDefinition(torsoName);
      torsoBodyDefinition.setMass(10.0);
      torsoBodyDefinition.setMomentOfInertia(new MomentOfInertiaDefinition(0.75, 0.75, 1.0));

      GeometryDefinition torsoGeometryDefinition = new Ellipsoid3DDefinition(0.15, 0.15, torsoLength/2.0);
      VisualDefinition torsoVisualDefinition = new VisualDefinition(torsoGeometryDefinition, ColorDefinition.rgb(Color.RED.getRGB()));
      torsoBodyDefinition.addVisualDefinition(torsoVisualDefinition);

      return torsoBodyDefinition;
   }

   private static RigidBodyDefinition createThigh(String name)
   {
      RigidBodyDefinition thighBodyDefinition = new RigidBodyDefinition(name);
      thighBodyDefinition.setMass(1.0);
      thighBodyDefinition.setMomentOfInertia(new MomentOfInertiaDefinition(0.1, 0.1, 0.01));

      GeometryDefinition thighGeometryDefinition = new Ellipsoid3DDefinition(0.05, 0.05, thighLength/2.0);
      VisualDefinition thighVisualDefinition = new VisualDefinition(thighGeometryDefinition, ColorDefinition.rgb(Color.ORANGE.getRGB()));
      thighBodyDefinition.addVisualDefinition(thighVisualDefinition);

      return thighBodyDefinition;
   }

   private static RigidBodyDefinition createShin(String name)
   {
      RigidBodyDefinition shinBodyDefinition = new RigidBodyDefinition(name);
      shinBodyDefinition.setMass(1.0);
      shinBodyDefinition.setMomentOfInertia(new MomentOfInertiaDefinition(0.1, 0.1, 0.01));

      GeometryDefinition shinGeometryDefinition = new Ellipsoid3DDefinition(0.03, 0.03, shinLength/2.0);
      VisualDefinition shinVisualDefinition = new VisualDefinition(shinGeometryDefinition, ColorDefinition.rgb(Color.YELLOW.getRGB()));
      shinBodyDefinition.addVisualDefinition(shinVisualDefinition);

      return shinBodyDefinition;
   }
}
