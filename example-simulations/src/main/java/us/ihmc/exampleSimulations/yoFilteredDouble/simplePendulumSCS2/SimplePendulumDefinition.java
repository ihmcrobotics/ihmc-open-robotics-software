package us.ihmc.exampleSimulations.yoFilteredDouble.simplePendulumSCS2;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

public class SimplePendulumDefinition extends RobotDefinition
{
   private static final String PENDULUM = "pendulum";

   // Define the parameters of the robot 
   public static final double ROD_LENGTH = 1.0;
   public static final double ROD_RADIUS = 0.01;
   public static final double ROD_MASS = 0.00;
   public static final double TIP_RADIUS = 0.02;

   public static final double FULCRUM_RADIUS = 0.02;

   public static final double BALL_RADIUS = 0.05;
   public static final double BALL_MASS = 1.0;

   // I = mr^2 pendulum's resistance changes to its rotation in kg.m^2
   public static final double FULCRUM_MOMENT_OF_INERTIA_ABOUT_Y = BALL_MASS * ROD_LENGTH * ROD_LENGTH;

   // Initial state of the pendulum
   private double fulcrumInitialPositionDegrees = 90.0;
   private double fulcrumInitialPositionRadians = fulcrumInitialPositionDegrees * Math.PI / 180.0;
   private double fulcrumInitialVelocity = 0.0;

   private static final double DAMP = 0.3;

   /*
    * Some joint state variables. Allows SimplePendulumRobot to have access to and set joint
    * properties.
    */
   //   private final OneDoFJointDampingControllerDefinition jointDampingControllerDefinition = new OneDoFJointDampingControllerDefinition();

   public static String jointName = "FulcrumPin";

   // Define its constructor
   public SimplePendulumDefinition()
   {
      // Call parent class "RobotDefinition" constructor. The string PENDULUM will be the name of the robot.
      super(PENDULUM);
      // creates an instance of the class RobotDefinition named "pendulum" in the SCS2 system.

      // Create the top (fixed) link that serves as the base of the pendulum
      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
      setRootBodyDefinition(elevator);

      /*
       * The first parameter is the name of the joint and will be used in all the variables associated
       * with the joint. The second parameter "new Vector3d(0.0, 0.0, 1.5)" defines the offset of this
       * joint from the previous joint. Since we want to position the fulcrum of the pendulum at a height
       * of 1.5 meters above the ground, the default vector (0.0, 0.0, 1.5) will be used. The parameter
       * "Axis3D.Y" defines the axis of rotation for this pin joint.
       */
      OneDoFJointDefinition fulcrumPinJoint = new RevoluteJointDefinition(jointName, new Vector3D(0.0, 0.0, 1.5), Axis3D.Y);

      // Set damping for the joint 
      fulcrumPinJoint.setDamping(DAMP);

      // Attach this joint to the top link 
      elevator.getChildrenJoints().add(fulcrumPinJoint);

      /*
       * Sets initial state of the pin joint. The pendulum will start is course from a horizontal position
       * with no initial speed.
       */
      fulcrumPinJoint.setInitialJointState(fulcrumInitialPositionRadians, fulcrumInitialVelocity);

      // Setup the pendulum link with the ball at its end
      createPendulumLink("PendulumLink", fulcrumPinJoint, BALL_MASS, ROD_LENGTH, ROD_RADIUS, 0.0, FULCRUM_MOMENT_OF_INERTIA_ABOUT_Y, 0.0);

      // TODO fix issue in SCS2
      addControllerDefinition((input, output) -> new Controller()
      {

         @Override
         public void doControl()
         {
            output.getOneDoFJointOutput(fulcrumPinJoint.getName()).setEffort(0);
         }
      });
   }

   /**
    * Create the link for the pendulum robot.
    */
   private RigidBodyDefinition createPendulumLink(String name,
                                                  JointDefinition parentJoint,
                                                  double mass,
                                                  double length,
                                                  double radius,
                                                  double Ixx,
                                                  double Iyy,
                                                  double Izz)
   {

      // Generate pendulum Link object
      RigidBodyDefinition pendulumLink = new RigidBodyDefinition(name);

      // Sets link's physical properties
      /*
       * Sets the moment of inertia about the X,Y,Z axis. Note that the moment of inertia is defined about
       * the center of mass, so if set to zero, the link will be a point mass.
       */
      pendulumLink.getMomentOfInertia().setToDiagonal(Ixx, Iyy, Izz);
      // sets the mass of the link
      pendulumLink.setMass(mass);
      // Sets center of mass offset to be located at tip of rod
      pendulumLink.setCenterOfMassOffset(0.0, 0.0, -length);
      parentJoint.setSuccessor(pendulumLink);

      // Set up the material properties and 3D shapes
      MaterialDefinition redMaterial = new MaterialDefinition(ColorDefinitions.Red());
      MaterialDefinition blackMaterial = new MaterialDefinition(ColorDefinitions.Black());
      MaterialDefinition greenMaterial = new MaterialDefinition(ColorDefinitions.Chartreuse());
      GeometryDefinition sphere1 = new Sphere3DDefinition(TIP_RADIUS);
      GeometryDefinition sphere2 = new Sphere3DDefinition(BALL_RADIUS);
      GeometryDefinition cylinder = new Cylinder3DDefinition(length, radius);

      // Set up the position offsets for the different elements
      RigidBodyTransform linkPose = new RigidBodyTransform(new AxisAngle(), new Vector3D(0.0, 0.0, -0.5 * length));
      RigidBodyTransform linkTip = new RigidBodyTransform(new AxisAngle(), new Vector3D(0.0, 0.0, -length));

      // Create the different visual elements
      pendulumLink.addVisualDefinition(new VisualDefinition(sphere1, redMaterial));
      pendulumLink.addVisualDefinition(new VisualDefinition(linkPose, cylinder, blackMaterial));
      pendulumLink.addVisualDefinition(new VisualDefinition(linkTip, sphere2, greenMaterial));

      return pendulumLink;
   }
}
