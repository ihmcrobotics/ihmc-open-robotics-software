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

   public static final double torsoLength = 0.5;
   public static final double thighLength = 0.5;
   public static final double shinLength = 0.5;

   private final SideDependentList<String> hipPitchNames = new SideDependentList<>(leftHipPitchName, rightHipPitchName);
   private final SideDependentList<String> thighNames = new SideDependentList<>(leftThighName, rightThighName);
   public static final SideDependentList<String> kneeNames = new SideDependentList<>(leftKneeName, rightKneeName);
   private final SideDependentList<String> shinNames = new SideDependentList<>(leftShinName, rightShinName);

   private final SideDependentList<RevoluteJointDefinition> hipPitchJointDefinitions = new SideDependentList<>();

   private final PlanarJointDefinition floatingBaseDefinition;

   private final RigidBodyDefinition torsoBodyDefinition;

   public BWCPlanarWalkingRobotDefinition()
   {
      setName(getClass().getSimpleName());

      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
      setRootBodyDefinition(elevator);
      floatingBaseDefinition = new PlanarJointDefinition("floatingBase");
      elevator.addChildJoint(floatingBaseDefinition);

      torsoBodyDefinition = createTorso();
      floatingBaseDefinition.setSuccessor(torsoBodyDefinition);

      for (RobotSide robotSide : RobotSide.values)
      {
         // Create the hip pitch joints and add them to the tree
         Vector3D hipPitchOffsetTorso = new Vector3D(0.0, robotSide.negateIfRightSide(0.05), -torsoLength / 2.0);
         RevoluteJointDefinition hipPitchJointDefinition = new RevoluteJointDefinition(hipPitchNames.get(robotSide), hipPitchOffsetTorso, Axis3D.Y);
         torsoBodyDefinition.addChildJoint(hipPitchJointDefinition);

         // Create the upper leg links and add them to the tree
         // FIXME: we probably need to add an offset so the link isn't placed at the origin
         RigidBodyDefinition thighLink = createThigh(thighNames.get(robotSide));
         hipPitchJointDefinition.setSuccessor(thighLink);

         // Create the knee joints and add them to the tree
         Vector3D kneeOffsetInThigh = new Vector3D(0.0, 0.0, -thighLength / 2.0);
         PrismaticJointDefinition kneeJointDefinition = new PrismaticJointDefinition(kneeNames.get(robotSide), kneeOffsetInThigh, Axis3D.Z);
         thighLink.addChildJoint(kneeJointDefinition);

         // Create the lower leg links and add them to the tree
         // FIXME: we probably need to add an offset so the link isn't placed at the origin
         RigidBodyDefinition shinLink = createShin(shinNames.get(robotSide));
         kneeJointDefinition.setSuccessor(shinLink);

         // Create the contact points for the feet
         GroundContactPointDefinition footContactPoint = new GroundContactPointDefinition(robotSide.getLowerCaseName() + "_gc_point", new Vector3D(0.0, 0.0, -shinLength / 2.0));
         kneeJointDefinition.addGroundContactPointDefinition(footContactPoint);
      }

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
