package us.ihmc.commonWalkingControlModules.contact.particleFilter;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.hamcrest.core.Is;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.simulationconstructionset.PinJoint;

/*package-private*/ class DoublePendulumRobotDefinition extends RobotDefinition
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double gravity = -9.81;
   private static final double linkLength1 = 0.7;
   private static final double lengthCoM1 = 0.3, lengthCoM2 = 0.25;
   private static final double Ixx1CoM = 0.4, Ixx2CoM = 0.5;
   private static final double Ismall = 1e-4;
   private static final double mass1 = 1.0, mass2 = 1.5;
   private static final double damping1 = 0.35, damping2 = 0.2;
   private static final Vector3D comOffset1 = new Vector3D(0.0, 0.0, -lengthCoM1);
   private static final Vector3D comOffset2 = new Vector3D(0.0, 0.0, -lengthCoM2);

   private final double dt;

   private final RigidBodyDefinition elevator;
   private final RigidBodyDefinition link1;
   private final RigidBodyDefinition link2;
   private final RevoluteJointDefinition joint1;
   private final RevoluteJointDefinition joint2;

   public static final String link1Name = "link1";
   public static final String link2Name = "link2";

   public static final String joint1Name = "joint1";
   public static final String joint2Name = "joint2";

   /**
    * Manipulator equation matrices, see {@link #updateManipulatorMatrices()}
    */
   private final DMatrixRMaj H = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj C = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj G = new DMatrixRMaj(2, 1);
   private final DMatrixRMaj qd = new DMatrixRMaj(2, 1);
   private final DMatrixRMaj Hdot = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj Hprev = new DMatrixRMaj(2, 2);

   DoublePendulumRobotDefinition(String name, double dt)
   {
      super(name);
      this.dt = dt;

      // setup id robot
      elevator = new RigidBodyDefinition("elevator");
      link1 = new RigidBodyDefinition(link1Name);
      link1.setMass(mass1);
      link1.getMomentOfInertia().setPrincipalComponents(Ixx1CoM, Ismall, Ismall);
      link1.setCenterOfMassOffset(comOffset1);
      VisualDefinitionFactory visualDefinitionFactory = new VisualDefinitionFactory();
      visualDefinitionFactory.addCylinder(linkLength1, 0.02, ColorDefinitions.Orange());
      link1.addVisualDefinitions(visualDefinitionFactory.getVisualDefinitions());

      link2 = new RigidBodyDefinition(link2Name);
      link2.setMass(mass2);
      link2.getMomentOfInertia().setPrincipalComponents(Ixx2CoM, Ismall, Ismall);
      link2.setCenterOfMassOffset(comOffset2);
      visualDefinitionFactory = new VisualDefinitionFactory();
      visualDefinitionFactory.addCylinder(linkLength1, 0.02, ColorDefinitions.Orange());
      link2.addVisualDefinitions(visualDefinitionFactory.getVisualDefinitions());

      joint1 = new RevoluteJointDefinition(joint1Name, new Vector3D(), Axis3D.X);
      joint2 = new RevoluteJointDefinition(joint2Name, new Vector3D(0.0, 0.0, -linkLength1), Axis3D.X);

      setRootBodyDefinition(elevator);
      elevator.addChildJoint(joint1);
      joint1.setSuccessor(link1);
      link1.addChildJoint(joint2);
      joint2.setSuccessor(link2);
   }

   /**
    * Solves the equations of motions outlined here:
    * @see <a href="https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-832-underactuated-robotics-spring-2009/readings/MIT6_832s09_read_ch03.pdf</a>
    */
   public void updateManipulatorMatrices(double q1, double qd1, double q2, double qd2)
   {
      this.Hprev.set(H);

      double g = Math.abs(gravity);
      double Ixx1 = mass1 * MathTools.square(lengthCoM1) + Ixx1CoM;
      double Ixx2 = mass2 * MathTools.square(lengthCoM2) + Ixx2CoM;

      double H00 = Ixx1 + Ixx2 + mass2 * MathTools.square(linkLength1) + 2.0 * mass2 * linkLength1 * lengthCoM2 * Math.cos(q2);
      double H01 = Ixx2 + mass2 * linkLength1 * lengthCoM2 * Math.cos(q2);
      double H10 = H01;
      double H11 = Ixx2;

      double C00 = -2.0 * mass2 * linkLength1 * lengthCoM2 * Math.sin(q2) * qd2 + damping1;
      double C01 = - mass2 * linkLength1 * lengthCoM2 * Math.sin(q2) * qd2;
      double C10 = mass2 * linkLength1 * lengthCoM2 * Math.sin(q2) * qd1;
      double C11 = damping2;

      double G00 = (mass1 * lengthCoM1 + mass2 * linkLength1) * g * Math.sin(q1) + mass2 * g * lengthCoM2 * Math.sin(q1 + q2);
      double G10 = mass2 * g * lengthCoM2 * Math.sin(q1 + q2);

      H.set(0, 0, H00);
      H.set(0, 1, H01);
      H.set(1, 0, H10);
      H.set(1, 1, H11);

      C.set(0, 0, C00);
      C.set(0, 1, C01);
      C.set(1, 0, C10);
      C.set(1, 1, C11);

      G.set(0, 0, G00);
      G.set(1, 0, G10);

      qd.set(0, 0, qd1);
      qd.set(1, 0, qd2);

      CommonOps_DDRM.subtract(H, Hprev, Hdot);
      CommonOps_DDRM.scale(1.0 / dt, Hdot);
   }

   public RevoluteJointDefinition getJoint2Definition()
   {
      return joint2;
   }

   public void setInitialState(double q1, double qd1, double q2, double qd2)
   {
      joint1.setInitialJointState(q1, qd1);
      joint2.setInitialJointState(q2, qd2);
   }
}