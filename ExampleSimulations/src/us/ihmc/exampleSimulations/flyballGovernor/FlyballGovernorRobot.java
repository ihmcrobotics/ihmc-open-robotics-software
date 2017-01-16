package us.ihmc.exampleSimulations.flyballGovernor;

import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.CylinderJoint;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;

@SuppressWarnings("unused")
public class FlyballGovernorRobot extends Robot implements RobotController
{
   private static final long serialVersionUID = -2657468625455223170L;

   private static final double L1 = 0.30, R1 = 0.01, M1 = 0.1, DAMPING1 = 0.0004;
   private static final double L2 = 0.2, R2 = 0.005, M2 = 0.5, SPHERE_R = 0.03, DAMPING2 = 0.5;
   private static final double L3 = 0.1, R3 = 0.005, M3 = 0.1;
   private static final double L4 = 0.06, R4 = 0.03, M4 = 0.2;

   private static final double Ixx1 = 0.5 * M1 * L1 * L1, Iyy1 = 0.5 * M1 * L1 * L1, Izz1 = 0.5 * M1 * R1 * R1;
   private static final double Ixx2 = 0.5 * M2 * SPHERE_R * SPHERE_R, Iyy2 = 0.5 * M2 * SPHERE_R * SPHERE_R, Izz2 = 0.5 * M2 * SPHERE_R * SPHERE_R;
   private static final double Ixx3 = 0.5 * M3 * L3 * L3, Iyy3 = 0.5 * M3 * L3 * L3, Izz3 = 0.5 * M3 * R3 * R3;
   private static final double Ixx4 = 0.5 * M4 * L4 * L4, Iyy4 = 0.5 * M4 * L4 * L4, Izz4 = 0.5 * M4 * R4 * R4;
   
   private final ExternalForcePoint constraint1A, constraint1B, constraint2A, constraint2B;

   public FlyballGovernorRobot(String nameSuffix, FlyballGovernorCommonControllerParameters controllerParameters, Vector3d baseWorldOffset)
   {
      super("FlyballGovernor" + nameSuffix);

      PinJoint rotation = new PinJoint("rotation", baseWorldOffset, this, Axis.Z);
      rotation.setDamping(DAMPING1);
      Link centerRod = centerRod();
      rotation.setLink(centerRod);
      this.addRootJoint(rotation);

      // One of the flyballs

      PinJoint upperPivot1 = new PinJoint("upperPivot1", new Vector3d(R1, 0.0, L1), this, Axis.Y);
      upperPivot1.setInitialState(-0.3, 0.0);
      upperPivot1.setDamping(DAMPING2);
      upperPivot1.setLimitStops(-Math.PI / 2.0, -0.2, 100, 10);
      Link flyBall1 = flyBallLink();
      upperPivot1.setLink(flyBall1);
      rotation.addJoint(upperPivot1);

      PinJoint lowerPivot1 = new PinJoint("lowerPivot1", new Vector3d(0.0, 0.0, -2.0 / 3.0 * L2), this, Axis.Y);
      lowerPivot1.setInitialState(0.5, 0.0);
      Link loopLink1 = loopLink();
      lowerPivot1.setLink(loopLink1);
      upperPivot1.addJoint(lowerPivot1);

      constraint1A = new ExternalForcePoint("constraint1A", new Vector3d(0.0, 0.0, -L3), this);
      lowerPivot1.addExternalForcePoint(constraint1A);

      // The other flyball

      PinJoint upperPivot2 = new PinJoint("upperPivot2", new Vector3d(-R1, 0.0, L1), this, Axis.Y);
      upperPivot2.setInitialState(0.3, 0.0);
      upperPivot2.setDamping(DAMPING2);
      upperPivot2.setLimitStops(0.2, Math.PI / 2.0, 100, 10);
      Link flyBall2 = flyBallLink();
      upperPivot2.setLink(flyBall2);
      rotation.addJoint(upperPivot2);

      PinJoint lowerPivot2 = new PinJoint("lowerPivot2", new Vector3d(0.0, 0.0, -2.0 / 3.0 * L2), this, Axis.Y);
      lowerPivot2.setInitialState(-0.5, 0.0);
      Link loopLink2 = loopLink();
      lowerPivot2.setLink(loopLink2);
      upperPivot2.addJoint(lowerPivot2);

      constraint2A = new ExternalForcePoint("constraint2A", new Vector3d(0.0, 0.0, -L3), this);
      lowerPivot2.addExternalForcePoint(constraint2A);


      // The sliding Cylinder:

      CylinderJoint cylinderJoint = new CylinderJoint("cylinder_theta", "cylinder_z", baseWorldOffset, this, Axis.Z);
      cylinderJoint.setInitialState(0.0, 0.0, 0.05, 0.0);
      Link cylinderLink = cylinderLink();
      cylinderJoint.setLink(cylinderLink);
      this.addRootJoint(cylinderJoint);

      constraint1B = new ExternalForcePoint("constraint1B", new Vector3d(R4, 0.0, L4 / 2.0), this);
      constraint2B = new ExternalForcePoint("constraint2B", new Vector3d(-R4, 0.0, L4 / 2.0), this);
      cylinderJoint.addExternalForcePoint(constraint1B);
      cylinderJoint.addExternalForcePoint(constraint2B);

      this.setController(this);
      
      k_feedback = controllerParameters.getK_feedback();
      q_d_cylinder_z = controllerParameters.getQ_d_cylinder_z();
      this.initControl();
   }

   public ExternalForcePoint getConstraint1A()
   {
      return constraint1A;
   }

   public ExternalForcePoint getConstraint1B()
   {
      return constraint1B;
   }

   public ExternalForcePoint getConstraint2A()
   {
      return constraint2A;
   }

   public ExternalForcePoint getConstraint2B()
   {
      return constraint2B;
   }

   private Link centerRod()
   {
      Link ret = new Link("Center Rod");
      ret.setMass(M1);
      ret.setMomentOfInertia(0.0, 0.0, Izz1);
      ret.setComOffset(0.0, 0.0, L1 / 2.0);
      
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCylinder(L1, R1, YoAppearance.Red());
      ret.setLinkGraphics(linkGraphics);
      
      return ret;
   }

   private Link flyBallLink()
   {
      Link ret = new Link("Flyball");
      ret.setMass(M2);
      ret.setMomentOfInertia(Ixx2, Iyy2, Izz2);
      ret.setComOffset(0.0, 0.0, -L2);
      
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, -L2);
      linkGraphics.addCylinder(L2, R2);
      linkGraphics.addSphere(SPHERE_R, YoAppearance.DarkGreen());
      ret.setLinkGraphics(linkGraphics);
      
      return ret;
   }


   private Link loopLink()
   {
      Link ret = new Link("Loop");
      ret.setMass(M3);
      ret.setMomentOfInertia(Ixx3, Iyy3, Izz3);
      ret.setComOffset(0.0, 0.0, -L3 / 2.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, -L3);
      linkGraphics.addCylinder(L3, R3);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link cylinderLink()
   {
      Link ret = new Link("Cylinder Link");
      ret.setMass(M4);
      ret.setMomentOfInertia(Ixx4, Iyy4, Izz4);
      ret.setComOffset(0.0, 0.0, L4 / 2.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCylinder(L4, R4, YoAppearance.DarkBlue());
      linkGraphics.addCylinder(L4 / 8.0, 1.1 * R4);
      linkGraphics.translate(0.0, 0.0, 7.0 / 8.0 * L4);
      linkGraphics.addCylinder(L4 / 8.0, 1.1 * R4);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private DoubleYoVariable tau_rotation, q_cylinder_z, qd_cylinder_z;

   private final YoVariableRegistry registry = new YoVariableRegistry("FlyballGovernorController");
   
   private final DoubleYoVariable k_feedback, q_d_cylinder_z;

   public DoubleYoVariable[] getControlVars()
   {
      return new DoubleYoVariable[] {k_feedback, q_d_cylinder_z};
   }

   public void initControl()
   {
      tau_rotation = (DoubleYoVariable)this.getVariable("tau_rotation");
      q_cylinder_z = (DoubleYoVariable)this.getVariable("q_cylinder_z");
      qd_cylinder_z = (DoubleYoVariable)this.getVariable("qd_cylinder_z");
   }

   public void doControl()
   {
      tau_rotation.set(k_feedback.getDoubleValue() * (q_d_cylinder_z.getDoubleValue() - q_cylinder_z.getDoubleValue()));
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }
   
   public void initialize()
   {      
   }

   public String getDescription()
   {
      return getName();
   }


}
