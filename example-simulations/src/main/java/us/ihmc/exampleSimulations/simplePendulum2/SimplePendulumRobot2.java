package us.ihmc.exampleSimulations.simplePendulum2;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimplePendulumRobot2 extends Robot
{
   /**
    * Define the parameters of the robot.
    */

   public static final double FULCRUM_RADIUS = 0.02;

   public static final double ROD_LENGTH = 1.0;
   public static final double ROD_RADIUS = 0.01;
   public static final double ROD_MASS = 0.00;

   public static final double BALL_RADIUS = 0.05;
   public static final double BALL_MASS = 1.0;

   public static final double FULCRUM_MOMENT_OF_INERTIA_ABOUT_Y = BALL_MASS * ROD_LENGTH * ROD_LENGTH;

   /**
    * Initial state of pendulum
    */
   private double fulcrumInitialPositionDegrees = 90.0;
   private double fulcrumInitialPositionRadians = fulcrumInitialPositionDegrees * Math.PI / 180.0;
   private double fulcrumInitialVelocity = 0.0;

   /**
    * Some joint state variables
    */
   private YoDouble tau_fulcrum, q_fulcrum, qd_fulcrum; // Respecttively Torque, Position, Velocity

   public SimplePendulumRobot2()
   {
      // a. Call parent class "Robot" constructor. The string "SimplePendulum" will be the name of the robot.  
      super("pendulum");

      // b. Add a joint to the robot.
      PinJoint fulcrumPinJoint = new PinJoint("FulcrumPin", new Vector3D(0.0, 0.0, 1.5), this, Axis3D.Y);
      fulcrumPinJoint.setInitialState(fulcrumInitialPositionRadians, fulcrumInitialVelocity);
      fulcrumPinJoint.setLink(pendulumLink());
      fulcrumPinJoint.setDamping(0.1);

      q_fulcrum = fulcrumPinJoint.getQYoVariable();
      qd_fulcrum = fulcrumPinJoint.getQDYoVariable();
      tau_fulcrum = fulcrumPinJoint.getTauYoVariable();

      this.addRootJoint(fulcrumPinJoint);

   }

   private Link pendulumLink()
   {
      Link pendulumLink = new Link("PendulumLink");
      pendulumLink.setMomentOfInertia(0.0, FULCRUM_MOMENT_OF_INERTIA_ABOUT_Y, 0.0);
      pendulumLink.setMass(BALL_MASS);
      pendulumLink.setComOffset(0.0, 0.0, -ROD_LENGTH);

      Graphics3DObject pendulumGraphics = new Graphics3DObject();
      pendulumGraphics.addSphere(FULCRUM_RADIUS, YoAppearance.BlueViolet()); // add the pivot
      pendulumGraphics.translate(0.0, 0.0, -ROD_LENGTH); // translate the origin
      pendulumGraphics.addCylinder(ROD_LENGTH, ROD_RADIUS, YoAppearance.Black()); // add the rod
      pendulumGraphics.addSphere(BALL_RADIUS, YoAppearance.DarkRed()); // add the ball

      pendulumLink.setLinkGraphics(pendulumGraphics);

      return pendulumLink;
   }
   
   /**
    * Fulcrum's angular position in radians
    * @return angular position in radians
    */
   public double getFulcrumAngularPosition()
   {
      return q_fulcrum.getDoubleValue();
   }

   /**
    * Fulcrum's angular velocity in radians per seconds
    * @return angular velocity in radians per seconds
    */
   public double getFulcrumAngularVelocity()
   {
      return qd_fulcrum.getDoubleValue();
   }

   /**
    * Fulcrum's torque in Newton meter
    * @return Torque in Newton meter
    */
   public double getFulcrumTorque()
   {
      return tau_fulcrum.getDoubleValue();
   }

   /**
    * Set Fulcrum's torque in Newton meter
    * @return Torque in Newton meter
    */
   public void setFulcrumTorque(double tau)
   {
      this.tau_fulcrum.set(tau);
   }
}
