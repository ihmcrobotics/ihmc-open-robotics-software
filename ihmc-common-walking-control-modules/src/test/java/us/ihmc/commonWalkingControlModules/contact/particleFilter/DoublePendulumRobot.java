package us.ihmc.commonWalkingControlModules.contact.particleFilter;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;

/*package-private*/ class DoublePendulumRobot extends Robot
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double gravity = 9.81;
   private static final double linkLength1 = 0.7;
   private static final double lengthCoM1 = 0.3, lengthCoM2 = 0.25;
   private static final double Ixx1CoM = 0.4, Ixx2CoM = 0.5;
   private static final double Ismall = 1e-4;
   private static final double mass1 = 1.0, mass2 = 1.5;
   private static final double damping1 = 0.35, damping2 = 0.2;
   private static final Vector3D comOffset1 = new Vector3D(0.0, 0.0, -lengthCoM1);
   private static final Vector3D comOffset2 = new Vector3D(0.0, 0.0, -lengthCoM2);

   private final PinJoint scsJoint1;
   private final PinJoint scsJoint2;
   private final double dt;

   private final RigidBodyBasics elevator;
   private final RevoluteJoint joint1;
   private final RigidBodyBasics link1;
   private final RevoluteJoint joint2;
   private final RigidBodyBasics link2;

   /**
    * Manipulator equation matrices, see {@link #updateManipulatorMatrices()}
    */
   private final DMatrixRMaj H = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj C = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj G = new DMatrixRMaj(2, 1);
   private final DMatrixRMaj qd = new DMatrixRMaj(2, 1);
   private final DMatrixRMaj Hdot = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj Hprev = new DMatrixRMaj(2, 2);

   DoublePendulumRobot(String name, double dt)
   {
      super(name);
      this.dt = dt;
      this.setGravity(0.0, 0.0, -gravity);

      // setup id robot
      elevator = new RigidBody("elevator", worldFrame);
      joint1 = new RevoluteJoint("joint1", elevator, Axis3D.X);
      link1 = new RigidBody("link1", joint1, Ixx1CoM, Ismall, Ismall, mass1, comOffset1);
      joint2 = new RevoluteJoint("joint2", link1, new Vector3D(0.0, 0.0, -linkLength1), Axis3D.X);
      link2 = new RigidBody("link2", joint2, Ixx2CoM, Ismall, Ismall, mass2, comOffset2);

      // setup scs robot
      scsJoint1 = new PinJoint("joint1", new Vector3D(0.0, 0.0, 0.0), this, Axis3D.X);
      scsJoint2 = new PinJoint("joint2", new Vector3D(0.0, 0.0, -linkLength1), this, Axis3D.X);

      scsJoint1.setDamping(damping1);
      scsJoint2.setDamping(damping2);

      Link scsLink1 = new Link("link1");
      scsLink1.setMass(mass1);
      scsLink1.setMomentOfInertia(Ixx1CoM, 1e-4, 1e-4);
      scsLink1.setComOffset(comOffset1);
      Graphics3DObject graphics3DObject1 = new Graphics3DObject();
      graphics3DObject1.rotate(Math.PI, Axis3D.X);
      graphics3DObject1.addCylinder(linkLength1, 0.02, YoAppearance.Red());
      scsLink1.setLinkGraphics(graphics3DObject1);

      Link scsLink2 = new Link("link2");
      scsLink2.setMass(mass2);
      scsLink2.setMomentOfInertia(Ixx2CoM, 1e-4, 1e-4);
      scsLink2.setComOffset(comOffset2);
      Graphics3DObject graphics3DObject2 = new Graphics3DObject();
      graphics3DObject2.rotate(Math.PI, Axis3D.X);
      graphics3DObject2.addCylinder(linkLength1, 0.02, YoAppearance.Blue());
      scsLink2.setLinkGraphics(graphics3DObject2);

      addRootJoint(scsJoint1);
      scsJoint1.setLink(scsLink1);
      scsJoint1.addJoint(scsJoint2);
      scsJoint2.setLink(scsLink2);
   }

   public void setIDStateFromSCS()
   {
      this.joint1.setQ(scsJoint1.getQ());
      this.joint1.setQd(scsJoint1.getQD());
      this.joint1.setTau(scsJoint1.getTau());

      this.joint2.setQ(scsJoint2.getQ());
      this.joint2.setQd(scsJoint2.getQD());
      this.joint2.setTau(scsJoint2.getTau());

      elevator.updateFramesRecursively();
   }

   /**
    * Solves the equations of motions outlined here:
    * @see <a href="https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-832-underactuated-robotics-spring-2009/readings/MIT6_832s09_read_ch03.pdf</a>
    */
   public void updateManipulatorMatrices()
   {
      setIDStateFromSCS();

      this.Hprev.set(H);

      double q1 = scsJoint1.getQ();
      double qd1 = scsJoint1.getQD();

      double q2 = scsJoint2.getQ();
      double qd2 = scsJoint2.getQD();

      double g = Math.abs(gravityZ.getDoubleValue());
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

   public DMatrixRMaj getH()
   {
      return H;
   }

   public DMatrixRMaj getC()
   {
      return C;
   }

   public DMatrixRMaj getG()
   {
      return G;
   }

   public DMatrixRMaj getQd()
   {
      return qd;
   }

   public DMatrixRMaj getHdot()
   {
      return Hdot;
   }

   public PinJoint getScsJoint1()
   {
      return scsJoint1;
   }

   public PinJoint getScsJoint2()
   {
      return scsJoint2;
   }

   public RevoluteJoint getJoint1()
   {
      return joint1;
   }

   public RevoluteJoint getJoint2()
   {
      return joint2;
   }

   public void setInitialState(double q1, double qd1, double q2, double qd2)
   {
      scsJoint1.setQ(q1);
      scsJoint1.setQd(qd1);
      scsJoint2.setQ(q2);
      scsJoint2.setQd(qd2);
   }
}