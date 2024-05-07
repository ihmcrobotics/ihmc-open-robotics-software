package us.ihmc.exampleSimulations.planarWalker;

import javafx.geometry.Side;
import org.opencv.features2d.SIFT;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.Ellipsoid3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.robot.*;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

import java.awt.*;

public class BWCPlanarWalkingRobotDefinition extends RobotDefinition
{
   private static final double damping = 0.1;

   public static final String baseJointName = "floatingBase";
   public static final String torsoName = "torso";
   public static final String leftHipPitchName = "l_hip_pitch";
   public static final String rightHipPitchName = "r_hip_pitch";
   public static final String leftKneeName = "l_knee_pitch";
   public static final String rightKneeName = "r_knee_pitch";
   public static final String leftThighName = "l_thigh";
   public static final String rightThighName = "r_thigh";
   public static final String leftShinName = "l_shin";
   public static final String rightShinName = "r_shin";


   private static final double torsoHeight = 0.5;
   public static final double thighLength = 0.5;
   public static final double shinLength = 0.5;

   private final SideDependentList<String> hipPitchNames = new SideDependentList<>(leftHipPitchName, rightHipPitchName);
   private final SideDependentList<String> thighNames = new SideDependentList<>(leftThighName, rightThighName);
   public static final SideDependentList<String> kneeNames = new SideDependentList<>(leftKneeName, rightKneeName);
   public static final SideDependentList<String> hipNames = new SideDependentList<>(leftHipPitchName, rightHipPitchName);
   private final SideDependentList<String> shinNames = new SideDependentList<>(leftShinName, rightShinName);

   private final SideDependentList<RevoluteJointDefinition> hipPitchJointDefinitions = new SideDependentList<>();

   private final PlanarJointDefinition floatingBaseDefinition;

   private final RigidBodyDefinition torsoBodyDefinition;

   public BWCPlanarWalkingRobotDefinition()
   {
      setName(getClass().getSimpleName());

      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
      setRootBodyDefinition(elevator);

      floatingBaseDefinition = new PlanarJointDefinition(baseJointName);
      elevator.addChildJoint(floatingBaseDefinition);

      torsoBodyDefinition = createTorso();
      floatingBaseDefinition.setSuccessor(torsoBodyDefinition);

      // Head Definition
      RigidBodyDefinition head = createHead();
      RevoluteJointDefinition neckJoint = new RevoluteJointDefinition("neck", new Vector3D(0.0, 0.0, torsoHeight), Axis3D.X);
      neckJoint.setSuccessor(head);
      torsoBodyDefinition.addChildJoint(neckJoint);


      KinematicPointDefinition basePoseDefintion = new KinematicPointDefinition("_base");
      floatingBaseDefinition.addKinematicPointDefinition(basePoseDefintion);

      for (RobotSide robotSide : RobotSide.values)
      {
         // create the hip pitch joints and add them to the tree
         Vector3D hipPitchOffsetInTorso = new Vector3D(0.0, robotSide.negateIfRightSide(0.05), -torsoHeight / 2.0);
         RevoluteJointDefinition hipPitchJointDefinition = new RevoluteJointDefinition(hipPitchNames.get(robotSide), hipPitchOffsetInTorso, Axis3D.Y);
         hipPitchJointDefinition.setDamping(damping);
         torsoBodyDefinition.addChildJoint(hipPitchJointDefinition);
         hipPitchJointDefinitions.put(robotSide, hipPitchJointDefinition);

         // create the upper leg links and add them to the tree
         // FIXME we probably need to add an offset from the joint attachment to the origin of the link.
         RigidBodyDefinition thighLink = createThigh(thighNames.get(robotSide));
         hipPitchJointDefinition.setSuccessor(thighLink);

         KinematicPointDefinition hipPoseDefintion = new KinematicPointDefinition(robotSide.getLowerCaseName() + "_hip");
         hipPitchJointDefinition.addKinematicPointDefinition(hipPoseDefintion);

         // create the knee joints and add them to the tree
         Vector3D leftKneeOffsetInThigh = new Vector3D(0.0, 0.0, -thighLength / 2.0);
         PrismaticJointDefinition kneeJointDefinition = new PrismaticJointDefinition(kneeNames.get(robotSide), leftKneeOffsetInThigh, Axis3D.Z);
         kneeJointDefinition.setDamping(damping);
         thighLink.addChildJoint(kneeJointDefinition);

         // create the shin links and add them to the tree
         // FIXME we probably need to add an offset from the joint attachment to the origin of the link.
         RigidBodyDefinition lowerLeg = createShin(shinNames.get(robotSide));
         kneeJointDefinition.setSuccessor(lowerLeg);

         KinematicPointDefinition kneePoseDefintion = new KinematicPointDefinition(robotSide.getLowerCaseName() + "_knee");
         kneeJointDefinition.addKinematicPointDefinition(kneePoseDefintion);

         // create the contact points for the feet.
         GroundContactPointDefinition footContactPoint = new GroundContactPointDefinition(robotSide.getLowerCaseName() + "_gc_point", new Vector3D(0.0, 0.0, -shinLength / 2.0));
         kneeJointDefinition.addGroundContactPointDefinition(footContactPoint);
      }

      for (RobotSide robotSide : RobotSide.values) {
         // Create and attach arm
         RigidBodyDefinition arm = createArm(robotSide.getCamelCaseName() + "Arm");
         Vector3D shoulderOffset = new Vector3D(0.0, robotSide.negateIfRightSide(0.1), torsoHeight / 2.0);
         RevoluteJointDefinition shoulderJoint = new RevoluteJointDefinition(robotSide.getCamelCaseName() + "Shoulder", shoulderOffset, Axis3D.Y);
         shoulderJoint.setDamping(0.1);
         torsoBodyDefinition.addChildJoint(shoulderJoint);
         shoulderJoint.setSuccessor(arm);

         // Create and attach hand
         RigidBodyDefinition hand = createHand(robotSide.getCamelCaseName() + "Hand");
         Vector3D wristOffset = new Vector3D(0.0, 0.0, -0.4);  // Assuming arm length is 0.4
         RevoluteJointDefinition wristJoint = new RevoluteJointDefinition(robotSide.getCamelCaseName() + "Wrist", wristOffset, Axis3D.Y);
         wristJoint.setDamping(0.1);
         arm.addChildJoint(wristJoint);
         wristJoint.setSuccessor(hand);
      }

      // TODO add some kind of collisions. Could be a collision shape. Could be a contact point.
   }

   private static RigidBodyDefinition createTorso()
   {
      double torsoMass = 10.0;
      MomentOfInertiaDefinition torsoMomentOfInertia = new MomentOfInertiaDefinition(0.75, 0.75, 1.0);

      RigidBodyDefinition torsoBodyDefinition = new RigidBodyDefinition(torsoName);
      torsoBodyDefinition.setMass(torsoMass);
      torsoBodyDefinition.setMomentOfInertia(torsoMomentOfInertia);

      GeometryDefinition torsoGeometryDefinition = new Ellipsoid3DDefinition(0.15, 0.15, torsoHeight / 2.0);
      VisualDefinition torsoVisualDefinition = new VisualDefinition(torsoGeometryDefinition, ColorDefinition.rgb(Color.RED.getRGB()));

      torsoBodyDefinition.addVisualDefinition(torsoVisualDefinition);

      return torsoBodyDefinition;
   }

   private static RigidBodyDefinition createThigh(String name)
   {
      double thighMass = 1.0;
      MomentOfInertiaDefinition thighMomentOfInertia = new MomentOfInertiaDefinition(0.1, 0.1, 0.01);

      RigidBodyDefinition thighDefinition = new RigidBodyDefinition(name);
      thighDefinition.setMass(thighMass);
      thighDefinition.setMomentOfInertia(thighMomentOfInertia);

      GeometryDefinition thighGeometryDefinition = new Ellipsoid3DDefinition(0.05, 0.05, thighLength / 2.0);
      VisualDefinition thighVisualDefinition = new VisualDefinition(thighGeometryDefinition, ColorDefinition.rgb(Color.ORANGE.getRGB()));

      thighDefinition.addVisualDefinition(thighVisualDefinition);

      return thighDefinition;
   }

   private static RigidBodyDefinition createShin(String name)
   {
      double shinMass = 3.0;
      MomentOfInertiaDefinition shinMomentOfInertia = new MomentOfInertiaDefinition(0.1, 0.1, 0.01);

      RigidBodyDefinition shinDefinition = new RigidBodyDefinition(name);
      shinDefinition.setMass(shinMass);
      shinDefinition.setMomentOfInertia(shinMomentOfInertia);

      GeometryDefinition shinGeometryDefinition = new Ellipsoid3DDefinition(0.03, 0.03, shinLength / 2.0);
      VisualDefinition shinVisualDefinition = new VisualDefinition(shinGeometryDefinition, ColorDefinition.rgb(Color.YELLOW.getRGB()));

      shinDefinition.addVisualDefinition(shinVisualDefinition);

      return shinDefinition;
   }

   private RigidBodyDefinition createHead() {
      double headMass = 3.0;
      MomentOfInertiaDefinition headMOI = new MomentOfInertiaDefinition(0.1, 0.1, 0.1);

      RigidBodyDefinition headDefinition = new RigidBodyDefinition("head");
      headDefinition.setMass(headMass);
      headDefinition.setMomentOfInertia(headMOI);

      GeometryDefinition headGeometry = new Ellipsoid3DDefinition(0.1, 0.1, 0.15);
      VisualDefinition headVisual = new VisualDefinition(headGeometry, ColorDefinition.rgb(Color.BLUE.getRGB()));

      headDefinition.addVisualDefinition(headVisual);
      return headDefinition;
   }

   private RigidBodyDefinition createArm(String name) {
      double armMass = 2.0;
      MomentOfInertiaDefinition armMOI = new MomentOfInertiaDefinition(0.2, 0.2, 0.02);

      RigidBodyDefinition armDefinition = new RigidBodyDefinition(name);
      armDefinition.setMass(armMass);
      armDefinition.setMomentOfInertia(armMOI);

      GeometryDefinition armGeometry = new Cylinder3DDefinition(0.05, 0.4);  // Cylinder length and radius)
      VisualDefinition armVisual = new VisualDefinition(armGeometry, ColorDefinition.rgb(Color.GRAY.getRGB()));

      armDefinition.addVisualDefinition(armVisual);
      return armDefinition;
   }

   private RigidBodyDefinition createHand(String name) {
      double handMass = 0.5;
      MomentOfInertiaDefinition handMOI = new MomentOfInertiaDefinition(0.05, 0.05, 0.05);

      RigidBodyDefinition handDefinition = new RigidBodyDefinition(name);
      handDefinition.setMass(handMass);
      handDefinition.setMomentOfInertia(handMOI);

      GeometryDefinition handGeometry = new Ellipsoid3DDefinition(0.05, 0.05, 0.1);
      VisualDefinition handVisual = new VisualDefinition(handGeometry, ColorDefinition.rgb(Color.GREEN.getRGB()));

      handDefinition.addVisualDefinition(handVisual);
      return handDefinition;
   }

}
