package us.ihmc.exampleSimulations.planarWalker.BWC;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.definition.geometry.Ellipsoid3DDefinition;
import us.ihmc.scs2.definition.robot.*;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

import java.awt.*;

public class BWCPlanarWalkingRobotDefinition extends RobotDefinition
{
   // Names for everything
   public static final String torsoName = "torso";

   public static final String leftHipPitchName = "l_hip_pitch";
   public static final String rightHipPitchName = "r_hip_pitch";

   public static final String leftKneeName = "l_knee_pitch";
   public static final String rightKneeName = "r_knee_pitch";

   public static final String leftThighName = "l_thigh";
   public static final String rightThighName = "r_thigh";

   public static final String leftShinName = "l_shin";
   public static final String rightShinName = "r_shin";

   public static final double thighLength = 0.5;
   public static final double torsoHeight = 0.5;
   public static final double shinLength = 0.3;

   // Joints
   private final PlanarJointDefinition floatingBaseJointDefinition;

   private final RevoluteJointDefinition leftHipPitchJointDefinition;
   private final RevoluteJointDefinition rightHipPitchJointDefinition;

   private final PrismaticJointDefinition leftKneeJointDefinition;
   private final PrismaticJointDefinition rightKneeJointDefinition;

   // Links/Rigid Bodies
   private final RigidBodyDefinition elevator;

   private final RigidBodyDefinition torsoBodyDefinition;

   private final RigidBodyDefinition leftThighBodyDefinition;
   private final RigidBodyDefinition rightThighBodyDefinition;

   private final RigidBodyDefinition leftShinBodyDefinition;
   private final RigidBodyDefinition rightShinBodyDefinition;

   public BWCPlanarWalkingRobotDefinition()
   {
      setName(getClass().getSimpleName());
      elevator = new RigidBodyDefinition("elevator");
      setRootBodyDefinition(elevator);

      floatingBaseJointDefinition = new PlanarJointDefinition("floatingBaseJoint");
      elevator.addChildJoint(floatingBaseJointDefinition);

      torsoBodyDefinition = createTorso();
      floatingBaseJointDefinition.setSuccessor(torsoBodyDefinition);

      // Create hip pitch joints, add them to tree
      leftHipPitchJointDefinition = new RevoluteJointDefinition(leftHipPitchName, new Vector3D(0.0, 0.05, -0.2), Axis3D.Y);
      rightHipPitchJointDefinition = new RevoluteJointDefinition(rightHipPitchName, new Vector3D(0.0, -0.05, -0.2), Axis3D.Y);

      torsoBodyDefinition.addChildJoint(leftHipPitchJointDefinition);
      torsoBodyDefinition.addChildJoint(rightHipPitchJointDefinition);

      // Create Upper leg links, add them to tree
      // FIXME probs need offset from joint to origin of link
      leftThighBodyDefinition = createThigh(leftThighName);
      leftHipPitchJointDefinition.setSuccessor(leftThighBodyDefinition);

      rightThighBodyDefinition = createThigh(rightThighName);
      rightHipPitchJointDefinition.setSuccessor(rightThighBodyDefinition);

      // Create knee joints, add them to tree
      Vector3D leftKneeOffsetInThigh = new Vector3D(0.0, 0.0, -thighLength/2.0);
      leftKneeJointDefinition = new PrismaticJointDefinition(leftKneeName, leftKneeOffsetInThigh, Axis3D.Z);
      leftThighBodyDefinition.addChildJoint(leftKneeJointDefinition);

      Vector3D rightKneeOffsetInThigh = new Vector3D(0.0, 0.0, -thighLength/2.0);
      rightKneeJointDefinition = new PrismaticJointDefinition(rightKneeName, rightKneeOffsetInThigh, Axis3D.Z);
      rightThighBodyDefinition.addChildJoint(rightKneeJointDefinition);

      leftShinBodyDefinition = createShin(leftShinName);
      rightShinBodyDefinition = createShin(rightShinName);

      leftKneeJointDefinition.setSuccessor(leftShinBodyDefinition);
      rightKneeJointDefinition.setSuccessor(rightShinBodyDefinition);

      GroundContactPointDefinition contactPointDefinition = new GroundContactPointDefinition("gcp", new Vector3D(0.0, 0.0, -shinLength/2.0));
      leftKneeJointDefinition.addGroundContactPointDefinition(contactPointDefinition);
      rightKneeJointDefinition.addGroundContactPointDefinition(contactPointDefinition);
   }

   private RigidBodyDefinition createTorso()
   {
      double torsoMass = 10.0;
      MomentOfInertiaDefinition torsoMomentOfInertiaDefinition = new MomentOfInertiaDefinition(0.75, 0.75, 1.0);
      VisualDefinition torsoVisualDefinition = new VisualDefinition(new Ellipsoid3DDefinition(0.15, 0.15, torsoHeight/2.0), ColorDefinition.rgb(Color.MAGENTA.getRGB()));

      RigidBodyDefinition torsoBodyDefinition = new RigidBodyDefinition(torsoName);
      torsoBodyDefinition.setMass(torsoMass);
      torsoBodyDefinition.setMomentOfInertia(torsoMomentOfInertiaDefinition);
      torsoBodyDefinition.addVisualDefinition(torsoVisualDefinition);

      return torsoBodyDefinition;
   }

   private RigidBodyDefinition createThigh(String thighName)
   {
      double thighMass = 1.0;
      MomentOfInertiaDefinition thighMomentOfInertiaDefinition = new MomentOfInertiaDefinition(0.1, 0.1, 0.01);
      VisualDefinition thighVisualDefinition = new VisualDefinition(new Ellipsoid3DDefinition(0.05, 0.05, thighLength/2.0), ColorDefinition.rgb(Color.CYAN.getRGB()));

      RigidBodyDefinition thighBodyDefinition = new RigidBodyDefinition(thighName);
      thighBodyDefinition.setMass(thighMass);
      thighBodyDefinition.setMomentOfInertia(thighMomentOfInertiaDefinition);
      thighBodyDefinition.addVisualDefinition(thighVisualDefinition);

      return thighBodyDefinition;
   }

   private RigidBodyDefinition createShin(String thighName)
   {
      double shinMass = 1.0;
      MomentOfInertiaDefinition shinMomentOfInertiaDefinition = new MomentOfInertiaDefinition(0.1, 0.1, 0.01);
      VisualDefinition shinVisualDefinition = new VisualDefinition(new Ellipsoid3DDefinition(0.03, 0.03, shinLength/2.0), ColorDefinition.rgb(Color.green.getRGB()));

      RigidBodyDefinition shinBodyDefinition = new RigidBodyDefinition(thighName);
      shinBodyDefinition.setMass(shinMass);
      shinBodyDefinition.setMomentOfInertia(shinMomentOfInertiaDefinition);
      shinBodyDefinition.addVisualDefinition(shinVisualDefinition);

      return shinBodyDefinition;
   }
}


