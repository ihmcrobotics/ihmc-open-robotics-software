package us.ihmc.exampleSimulations.simplePendulum;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;

/**
 * In this tutorial, lengths are expressed in meters (m), masses in kilograms (kg)
 */
public class SimplePendulumRobotDefinition extends RobotDefinition
{
   /*
      Define the parameters of the robot
   */
   public static final double ROD_LENGTH = 1.0;
   public static final double ROD_RADIUS = 0.01;

   public static final double FULCRUM_RADIUS = 0.02;

   public static final double BALL_RADIUS = 0.05;
   public static final double BALL_MASS = 1.0;

   public static final double FULCRUM_MOMENT_OF_INERTIA_ABOUT_Y =
         ROD_LENGTH * ROD_LENGTH * BALL_MASS; // I = mrˆ2 pendulum's resistance to changes to its rotation in  kg.mˆ2

   /*
      Initial state of the pendulum
   */

   private double fulcrumInitialPositionDegrees = 90.0;
   private double fulcrumInitialPositionRadians = fulcrumInitialPositionDegrees * Math.PI / 180.0;
   private double fulcrumInitialVelocity = 0.0;

   /*
      Define its constructor
    */
   public SimplePendulumRobotDefinition()
   {
      // a. Call parent class "RobotDefinition" constructor. The string "SimplePendulum" will be the name of the robot.
      super("SimplePendulum");

      // b. SCS2 robots need an explicit (massless) root body that the first joint attaches to.
      RigidBodyDefinition rootBody = new RigidBodyDefinition("rootBody");
      setRootBodyDefinition(rootBody);

      // c. Add a joint to the robot
      RevoluteJointDefinition fulcrumPinJoint = new RevoluteJointDefinition("FulcrumPin", new Vector3D(0.0, 0.0, 1.0), Axis3D.Y);
      fulcrumPinJoint.setInitialJointState(fulcrumInitialPositionRadians, fulcrumInitialVelocity);
      fulcrumPinJoint.setDamping(0.3);
      fulcrumPinJoint.setSuccessor(pendulumLink()); // pendulumLink() method defined next.

      rootBody.addChildJoint(fulcrumPinJoint);

      // d. Wire up the controller.
      addControllerDefinition((controllerInput, controllerOutput) -> new SimplePendulumController(controllerInput, controllerOutput));
   }

   /**
    * Create the first link for the SimplePendulumRobot.
    */
   private RigidBodyDefinition pendulumLink()
   {
      RigidBodyDefinition pendulumLink = new RigidBodyDefinition("PendulumLink");
      pendulumLink.setMomentOfInertia(new Matrix3D(0.0, 0.0, 0.0, 0.0, FULCRUM_MOMENT_OF_INERTIA_ABOUT_Y, 0.0, 0.0, 0.0, 0.0));
      pendulumLink.setMass(BALL_MASS);
      pendulumLink.setCenterOfMassOffset(0.0, 0.0, -ROD_LENGTH);

      VisualDefinitionFactory pendulumGraphics = new VisualDefinitionFactory();
      pendulumGraphics.addSphere(FULCRUM_RADIUS, ColorDefinitions.BlueViolet());
      pendulumGraphics.appendTranslation(0.0, 0.0, -ROD_LENGTH);
      pendulumGraphics.addCylinder(ROD_LENGTH, ROD_RADIUS, ColorDefinitions.Black());
      pendulumGraphics.addSphere(BALL_RADIUS, ColorDefinitions.Chartreuse());
      pendulumLink.addVisualDefinitions(pendulumGraphics.getVisualDefinitions());

      return pendulumLink;
   }
}
