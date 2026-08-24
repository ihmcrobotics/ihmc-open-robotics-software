package us.ihmc.exampleSimulations.beetle;

import us.ihmc.exampleSimulations.beetle.parameters.RhinoBeetleModelFactory;
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;

public class RhinoBeetleSDFLoadingDemo
{
   private static final boolean SHOW_ELLIPSOIDS = true;
   private static final boolean SHOW_COORDINATES_AT_JOINT_ORIGIN = false;

   public RhinoBeetleSDFLoadingDemo()
   {
      RhinoBeetleModelFactory robotParameters = new RhinoBeetleModelFactory();
      RobotDefinition robotDefinition = robotParameters.getRobotDefinition();

      SixDoFJointDefinition rootJoint = robotDefinition.getFloatingRootJointDefinition();
      rootJoint.getInitialJointState().getPosition().set(0.0, 0.0, 1.0);

      double totalMass = robotDefinition.getAllRigidBodies().stream().mapToDouble(RigidBodyDefinition::getMass).sum();
      System.out.println("Total robot mass: " + totalMass);

      if (SHOW_ELLIPSOIDS)
      {
         addInertialEllipsoidsToVisualizer(robotDefinition);
      }

      if (SHOW_COORDINATES_AT_JOINT_ORIGIN)
      {
         addJointAxis(robotDefinition);
      }

      SimulationSession simulationSession = new SimulationSession();
      simulationSession.addRobot(robotDefinition);

      SessionVisualizer.startSessionVisualizer(simulationSession);
   }

   private void addInertialEllipsoidsToVisualizer(RobotDefinition robotDefinition)
   {
      for (RigidBodyDefinition rigidBody : robotDefinition.getAllRigidBodies())
      {
         double mass = rigidBody.getMass();
         if (mass <= 0.0)
            continue;

         // Inertia-ellipsoid radii derived from the diagonal of the moment-of-inertia tensor (no principal-axis
         // rotation applied), same closed-form SCS1's Link.addEllipsoidFromMassProperties used for a solid ellipsoid.
         MomentOfInertiaDefinition inertia = rigidBody.getMomentOfInertia();
         double ixx = inertia.getM00();
         double iyy = inertia.getM11();
         double izz = inertia.getM22();

         double radiusX = Math.sqrt(2.5 * (-ixx + iyy + izz) / mass);
         double radiusY = Math.sqrt(2.5 * (ixx - iyy + izz) / mass);
         double radiusZ = Math.sqrt(2.5 * (ixx + iyy - izz) / mass);

         VisualDefinitionFactory linkGraphics = new VisualDefinitionFactory();
         linkGraphics.appendTranslation(rigidBody.getCenterOfMassOffset());
         linkGraphics.addEllipsoid(radiusX, radiusY, radiusZ, ColorDefinitions.Green().derive(0, 1, 1, 0.6));
         linkGraphics.identity();
         linkGraphics.appendTranslation(rigidBody.getCenterOfMassOffset());
         linkGraphics.addCoordinateSystem(0.1);

         rigidBody.addVisualDefinitions(linkGraphics.getVisualDefinitions());
      }
   }

   private void addJointAxis(RobotDefinition robotDefinition)
   {
      for (var joint : robotDefinition.getAllOneDoFJoints())
      {
         VisualDefinitionFactory linkGraphics = new VisualDefinitionFactory();
         linkGraphics.addCoordinateSystem(0.1);
         joint.getSuccessor().addVisualDefinitions(linkGraphics.getVisualDefinitions());
      }
   }

   public static void main(String[] args)
   {
      new RhinoBeetleSDFLoadingDemo();
   }
}
