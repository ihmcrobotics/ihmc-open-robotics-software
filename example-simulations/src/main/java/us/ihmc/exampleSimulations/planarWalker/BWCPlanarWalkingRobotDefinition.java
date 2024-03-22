package us.ihmc.exampleSimulations.planarWalker;

import javafx.geometry.Side;
import org.opencv.features2d.SIFT;
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

public class BWCPlanarWalkingRobotDefinition extends RobotDefinition
{
   private static final double damping = 0.1;

   public static final String baseJointName = "floatingBase";
   public static final String torsoName = "torso";
   public static final String leftHipPitchName = "l_hip_pitch";
   public static final String rightHipPitchName = "r_hip_pitch";
   public static final String leftHipRollName = "l_hip_roll";
   public static final String rightHipRollName = "r_hip_roll";
   public static final String leftKneeName = "l_knee_pitch";
   public static final String rightKneeName = "r_knee_pitch";
   public static final String leftHipLinkName = "l_hip_link";
   public static final String rightHipLinkName = "r_hip_link";
   public static final String leftThighName = "l_thigh";
   public static final String rightThighName = "r_thigh";
   public static final String leftShinName = "l_shin";
   public static final String rightShinName = "r_shin";


   private static final double torsoHeight = 0.5;
   public static final double thighLength = 0.5;
   public static final double shinLength = 0.5;

   public static final SideDependentList<String> hipPitchNames = new SideDependentList<>(leftHipPitchName, rightHipPitchName);
   public static final SideDependentList<String> hipLinkNames = new SideDependentList<>(leftHipLinkName, rightHipLinkName);
   public static final SideDependentList<String> hipRollNames = new SideDependentList<>(leftHipRollName, rightHipRollName);
   public static final SideDependentList<String> thighNames = new SideDependentList<>(leftThighName, rightThighName);
   public static final SideDependentList<String> kneeNames = new SideDependentList<>(leftKneeName, rightKneeName);
   public static final SideDependentList<String> shinNames = new SideDependentList<>(leftShinName, rightShinName);

   private final SideDependentList<RevoluteJointDefinition> hipPitchJointDefinitions = new SideDependentList<>();
   private final SideDependentList<RevoluteJointDefinition> hipRollJointDefinitions = new SideDependentList<>();

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

      KinematicPointDefinition basePoseDefintion = new KinematicPointDefinition("_base");
      floatingBaseDefinition.addKinematicPointDefinition(basePoseDefintion);

      for (RobotSide robotSide : RobotSide.values)
      {
         // create the hip roll joints and add them to the tree
         Vector3D hipRollOffsetInTorso = new Vector3D(0.0, robotSide.negateIfRightSide(0.05), -torsoHeight / 2.0);
         RevoluteJointDefinition hipRollJointDefinition = new RevoluteJointDefinition(hipRollNames.get(robotSide), hipRollOffsetInTorso, Axis3D.X);
         hipRollJointDefinition.setDamping(damping);
         hipRollJointDefinition.setPositionLimits(-Math.PI / 2.0, Math.PI / 2.0);
         torsoBodyDefinition.addChildJoint(hipRollJointDefinition);
         hipRollJointDefinitions.put(robotSide, hipRollJointDefinition);

         // create the hip link that connects the hip roll joint to the hip pitch joint and add it to the tree
         RigidBodyDefinition hipLink = createHipLink(hipLinkNames.get(robotSide));
         hipRollJointDefinition.setSuccessor(hipLink);

         KinematicPointDefinition hipRollPoseDefintion = new KinematicPointDefinition(robotSide.getLowerCaseName() + "_hipRoll");
         hipRollJointDefinition.addKinematicPointDefinition(hipRollPoseDefintion);

         // create the hip pitch joints and add them to the tree
         Vector3D hipPitchOffsetInTorso = new Vector3D(0.0, robotSide.negateIfRightSide(0.0), 0.0); //TODO: Verify this offset
         RevoluteJointDefinition hipPitchJointDefinition = new RevoluteJointDefinition(hipPitchNames.get(robotSide), hipPitchOffsetInTorso, Axis3D.Y);
         hipPitchJointDefinition.setDamping(damping);
         hipPitchJointDefinition.setPositionLimits(-Math.PI / 2.0, Math.PI / 2.0);
         hipLink.addChildJoint(hipPitchJointDefinition);
         hipPitchJointDefinitions.put(robotSide, hipPitchJointDefinition);

         // create the upper leg links and add them to the tree
         RigidBodyDefinition thighLink = createThigh(thighNames.get(robotSide));
         hipPitchJointDefinition.setSuccessor(thighLink);

         KinematicPointDefinition hipPitchPoseDefintion = new KinematicPointDefinition(robotSide.getLowerCaseName() + "_hipPitch");
         hipPitchJointDefinition.addKinematicPointDefinition(hipPitchPoseDefintion);

         // create the knee joints and add them to the tree
         Vector3D leftKneeOffsetInThigh = new Vector3D(0.0, 0.0, -thighLength / 2.0);
         PrismaticJointDefinition kneeJointDefinition = new PrismaticJointDefinition(kneeNames.get(robotSide), leftKneeOffsetInThigh, Axis3D.Z);
         kneeJointDefinition.setDamping(damping);
         kneeJointDefinition.setPositionLimits(0.0, shinLength / 2.0);
         thighLink.addChildJoint(kneeJointDefinition);

         // create the shin links and add them to the tree
         RigidBodyDefinition lowerLeg = createShin(shinNames.get(robotSide));
         kneeJointDefinition.setSuccessor(lowerLeg);

         KinematicPointDefinition kneePoseDefintion = new KinematicPointDefinition(robotSide.getLowerCaseName() + "_knee");
         kneeJointDefinition.addKinematicPointDefinition(kneePoseDefintion);

         // create the contact points for the feet.
         GroundContactPointDefinition footContactPoint = new GroundContactPointDefinition(robotSide.getLowerCaseName() + "_gc_point", new Vector3D(0.0, 0.0, -shinLength / 2.0));
         kneeJointDefinition.addGroundContactPointDefinition(footContactPoint);
      }
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

   //TODO: Verify this method is even necessary
   private static RigidBodyDefinition createHipLink(String name)
   {
      double hipLinkMass = 0.0;
      MomentOfInertiaDefinition hipLinkMomentOfInertia = new MomentOfInertiaDefinition(0.0, 0.0, 0.0);

      RigidBodyDefinition hipLinkDefinition = new RigidBodyDefinition(name);
      hipLinkDefinition.setMass(hipLinkMass);
      hipLinkDefinition.setMomentOfInertia(hipLinkMomentOfInertia);

//      GeometryDefinition thighGeometryDefinition = new Ellipsoid3DDefinition(0.05, 0.05, thighLength / 2.0);
//      VisualDefinition thighVisualDefinition = new VisualDefinition(thighGeometryDefinition, ColorDefinition.rgb(Color.ORANGE.getRGB()));
//
//      hipLinkDefinition.addVisualDefinition(thighVisualDefinition);

      return hipLinkDefinition;
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
}
