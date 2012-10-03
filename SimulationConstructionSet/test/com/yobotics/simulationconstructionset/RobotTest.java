package com.yobotics.simulationconstructionset;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.math.RotationalInertiaCalculator;
import us.ihmc.utilities.test.JUnitTools;

import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class RobotTest
{
   private static final double COORDINATE_SYSTEM_LENGTH = 0.3;
   private static final boolean SHOW_GUI = false;

   @Test
   public void testSwitchingRootJoint() throws InterruptedException, UnreasonableAccelerationException, SimulationExceededMaximumTimeException
   {
      Random random = new Random(1765L);
      double l1 = 2.0, l2 = 2.0, r1 = 0.1, r2 = 0.05;


      Robot robot1 = new Robot("r1");
      FloatingJoint root1 = new FloatingJoint("root1", new Vector3d(), robot1);
      robot1.addRootJoint(root1);
      Link link11 = link11(random, l1, r1);
      root1.setLink(link11);

      Vector3d offset = new Vector3d(0.0, 0.0, l1);

//    Vector3d offset = new Vector3d();
      PinJoint pin1 = new PinJoint("pin1", offset, robot1, Joint.Y);
      root1.addJoint(pin1);
      Link link21 = link21(random, l2, r2);
      pin1.setLink(link21);

      Robot robot2 = new Robot("r2");
      FloatingJoint root2 = new FloatingJoint("root2", new Vector3d(), robot2);
      robot2.addRootJoint(root2);
      Link link22 = link22(link21);
      root2.setLink(link22);
      Vector3d jointAxis = new Vector3d();
      pin1.getJointAxis(jointAxis);
      PinJoint pin2 = new PinJoint("pin2", new Vector3d(), robot2, jointAxis);
      root2.addJoint(pin2);
      Link link12 = link12(link11, pin1);
      pin2.setLink(link12);

      robot1.setGravity(0.0);
      robot2.setGravity(0.0);

//    System.out.println(new RobotExplorer(robot1));
//    System.out.println("\n\n\n");
//    System.out.println(new RobotExplorer(robot2));
//    System.out.println("\n\n\n");

      pin1.setQ(random.nextDouble());
      pin2.setQ(-pin1.getQ().getDoubleValue());

      pin1.setQd(10.0);
      pin2.setQd(-pin1.getQD().getDoubleValue());
      Vector3d angularVelocityInBody = new Vector3d(0.0, pin1.getQD().getDoubleValue(), 0.0);
      root2.setAngularVelocityInBody(angularVelocityInBody);

      pin1.setTau(random.nextDouble());
      pin2.setTau(-pin1.getTau().getDoubleValue());

      robot1.update();
      Transform3D pin1ToWorld = new Transform3D();
      pin1.getTransformToWorld(pin1ToWorld);
      root2.setRotationAndTranslation(pin1ToWorld);
      root2.setPosition(root2.getQx().getDoubleValue(), root2.getQy().getDoubleValue(), root2.getQz().getDoubleValue());
      robot2.update();

      robot1.updateVelocities();
      robot2.updateVelocities();

//    System.out.println("com1: " + computeCoM(robot1));
//    System.out.println("com2: " + computeCoM(robot2));
//
//    System.out.println("linearMomentum1: " + computeLinearMomentum(robot1));
//    System.out.println("linearMomentum2: " + computeLinearMomentum(robot2));
//
//    System.out.println("angularMomentum1: " + computeAngularMomentum(robot1));
//    System.out.println("angularMomentum2: " + computeAngularMomentum(robot2));
//    
//    System.out.println("scalar inertia 11: " + computeScalarInertiaAroundJointAxis(link11, pin1));
//    System.out.println("scalar inertia 12: " + computeScalarInertiaAroundJointAxis(link12, pin2));
//    
//    System.out.println("scalar inertia 21: " + computeScalarInertiaAroundJointAxis(link21, pin1));
//    System.out.println("scalar inertia 22: " + computeScalarInertiaAroundJointAxis(link22, pin2));

      double epsilonBefore = 1e-8;
      JUnitTools.assertTuple3dEquals(computeCoM(robot1), computeCoM(robot2), epsilonBefore);
      JUnitTools.assertTuple3dEquals(computeLinearMomentum(robot1), computeLinearMomentum(robot2), epsilonBefore);
      JUnitTools.assertTuple3dEquals(computeAngularMomentum(robot1), computeAngularMomentum(robot2), epsilonBefore);
      assertEquals(computeScalarInertiaAroundJointAxis(link11, pin1), computeScalarInertiaAroundJointAxis(link12, pin2), epsilonBefore);
      assertEquals(computeScalarInertiaAroundJointAxis(link21, pin1), computeScalarInertiaAroundJointAxis(link22, pin2), epsilonBefore);

      robot1.doDynamicsButDoNotIntegrate();
      robot2.doDynamicsButDoNotIntegrate();

      assertEquals(pin1.getQDD().getDoubleValue(), pin1.getQDD().getDoubleValue(), 1e-8);

      double simulateTime = 1.0;
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot[] {robot1, robot2}, SHOW_GUI);
      scs.setDT(1e-4, 10);
      scs.setGroundVisible(false);
      Thread simThread = new Thread(scs, "sim thread");
      simThread.start();
      BlockingSimulationRunner simulationRunner = new BlockingSimulationRunner(scs, 50.0);
      simulationRunner.simulateAndBlock(simulateTime);
      sleepIfShowingGUI();

      double epsilonAfter = 1e-4;
      JUnitTools.assertTuple3dEquals(computeCoM(robot1), computeCoM(robot2), epsilonAfter);
      JUnitTools.assertTuple3dEquals(computeLinearMomentum(robot1), computeLinearMomentum(robot2), epsilonAfter);
      JUnitTools.assertTuple3dEquals(computeAngularMomentum(robot1), computeAngularMomentum(robot2), epsilonAfter);
      assertEquals(computeScalarInertiaAroundJointAxis(link11, pin1), computeScalarInertiaAroundJointAxis(link12, pin2), epsilonAfter);
      assertEquals(computeScalarInertiaAroundJointAxis(link21, pin1), computeScalarInertiaAroundJointAxis(link22, pin2), epsilonAfter);
   }

   @Test
   public void testSingleFloatingBodyWithCoMOffset() throws SimulationExceededMaximumTimeException, InterruptedException, UnreasonableAccelerationException
   {
      Random random = new Random(1659L);
      Robot robot = new Robot("r1");
      robot.setGravity(0.0);

      FloatingJoint root1 = new FloatingJoint("root1", new Vector3d(), robot);
      robot.addRootJoint(root1);

      Link floatingBody = randomBody(random);
      root1.setLink(floatingBody);
      Vector3d comOffset = floatingBody.getComOffset();

      Vector3d externalForcePointOffset = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
      ExternalForcePoint externalForcePoint = new ExternalForcePoint("efp", externalForcePointOffset, robot);
      root1.addExternalForcePoint(externalForcePoint);
      Vector3d force = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
      externalForcePoint.setForce(force);

      robot.doDynamicsButDoNotIntegrate();

      // EULER
      // moment about CoM
      Transform3D transformToWorld = new Transform3D();
      root1.getTransformToWorld(transformToWorld);
      Transform3D transformToBody = new Transform3D();
      transformToBody.invert(transformToWorld);
      transformToBody.transform(force);
      Vector3d moment = new Vector3d();
      moment.sub(externalForcePointOffset, comOffset);
      moment.cross(moment, force);

      // moment from dynamics
      Matrix3d momentOfInertia = new Matrix3d();
      floatingBody.getMomentOfInertia(momentOfInertia);

      Vector3d temp1 = new Vector3d();
      temp1.set(root1.getAngularAccelerationInBody());
      momentOfInertia.transform(temp1);    // J omegad

      Vector3d temp2 = new Vector3d();
      temp2.set(root1.getAngularVelocityInBody());
      momentOfInertia.transform(temp2);    // J omega
      temp2.cross(root1.getAngularVelocityInBody(), temp2);    // omega x J omega

      Vector3d angularMomentumDerivative = new Vector3d();
      angularMomentumDerivative.add(temp1, temp2);    // J omegad + omega x J omega
      JUnitTools.assertTuple3dEquals(angularMomentumDerivative, moment, 1e-10);

      // NEWTON
      Vector3d linearMomentum0 = computeLinearMomentum(robot);

      double dt = 1e-8;
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, SHOW_GUI);
      scs.setDT(1e-10, 10);
      scs.setGroundVisible(false);
      Thread simThread = new Thread(scs, "sim thread");
      simThread.start();
      BlockingSimulationRunner simulationRunner = new BlockingSimulationRunner(scs, 50.0);
      simulationRunner.simulateAndBlock(dt);

      Vector3d linearMomentumDT = computeLinearMomentum(robot);
      Vector3d linearMomentumDerivativeNumerical = new Vector3d();
      linearMomentumDerivativeNumerical.sub(linearMomentumDT, linearMomentum0);
      linearMomentumDerivativeNumerical.scale(1.0 / dt);
      JUnitTools.assertTuple3dEquals(linearMomentumDerivativeNumerical, force, 1e-10);

//    sleepIfShowingGUI();
   }

   @Test
   public void testFloatingJointAndPinJointWithMassiveBody() throws UnreasonableAccelerationException
   {
      Random random = new Random(1659L);
      Robot robot = new Robot("r1");
      robot.setGravity(0.0);

      FloatingJoint root1 = new FloatingJoint("root1", new Vector3d(), robot);
      robot.addRootJoint(root1);

      Link floatingBody = new Link("floatingBody");
      floatingBody.setMass(random.nextDouble());

      floatingBody.setComOffset(random.nextDouble(), random.nextDouble(), random.nextDouble());
      floatingBody.setMomentOfInertia(RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidEllipsoid(floatingBody.getMass(), random.nextDouble(),
              random.nextDouble(), random.nextDouble()));
      root1.setLink(floatingBody);

      Vector3d offset = RandomTools.getRandomVector(random);
      PinJoint pin1 = new PinJoint("pin1", offset, robot, RandomTools.getRandomVector(random));
      pin1.setLink(massiveLink());
      root1.addJoint(pin1);

      pin1.setTau(random.nextDouble());
      robot.doDynamicsButDoNotIntegrate();

      double scalarInertiaAboutJointAxis = computeScalarInertiaAroundJointAxis(floatingBody, pin1);

      double torqueFromDynamics = scalarInertiaAboutJointAxis * pin1.getQDD().getDoubleValue();
      assertEquals(pin1.getTau().getDoubleValue(), torqueFromDynamics, pin1.getTau().getDoubleValue() * 1e-3);
   }

   @Test
   public void testCompareFloatingJointAndFLoatingPlanarJoint() throws UnreasonableAccelerationException, SimulationExceededMaximumTimeException, InterruptedException
   {
      long seed = 101L;
      Random random = new Random(seed);
      double gravity = 0.0; // random.nextDouble();

      Vector3d offset = new Vector3d(random.nextDouble(), 0.0, random.nextDouble());

      Robot robot1 = new Robot("r1");
      FloatingJoint floatingJoint1 = new FloatingJoint("joint1", new Vector3d(), robot1);
      robot1.addRootJoint(floatingJoint1);
      floatingJoint1.setLink(randomBodyNoYCoMOffset(random));
      PinJoint pin1 = new PinJoint("pin1", offset, robot1, Joint.Y);
      floatingJoint1.addJoint(pin1);
      pin1.setLink(randomBodyNoYCoMOffset(random));

      Robot robot2 = new Robot("r2");
      FloatingPlanarJoint floatingJoint2 = new FloatingPlanarJoint("joint2", robot2);
      robot2.addRootJoint(floatingJoint2);
      floatingJoint2.setLink(new Link(floatingJoint1.getLink()));
      PinJoint pin2 = new PinJoint("pin2", offset, robot2, Joint.Y);
      floatingJoint2.addJoint(pin2);
      pin2.setLink(new Link(pin1.getLink()));

      Robot[] robots = new Robot[] {robot1, robot2};
      PinJoint[] pinJoints = new PinJoint[]{pin1, pin2};
      for (Robot robot : robots)
      {
//         System.out.println(new RobotExplorer(robot));
         robot.setGravity(gravity);
      }
      
      double q = random.nextDouble();
      double tau = random.nextDouble();
      for (PinJoint pinJoint : pinJoints)
      {
         pinJoint.setQ(q);
         pinJoint.setTau(tau);
      }

      for (Robot robot : robots)
      {
         robot.doDynamicsButDoNotIntegrate();
      }

//      double simulateTime = 2.0;
//      SimulationConstructionSet scs = new SimulationConstructionSet(robot1, SHOW_GUI);
//      scs.setDT(1e-4, 10);
//      scs.setGroundVisible(false);
//      Thread simThread = new Thread(scs, "sim thread");
//      simThread.start();
//      BlockingSimulationRunner simulationRunner = new BlockingSimulationRunner(scs, 50.0);
//      simulationRunner.simulateAndBlock(simulateTime);
//      sleepIfShowingGUI();
      
//      System.out.println(floatingJoint1.getQddx() + ", " + floatingJoint2.getQdd_t1());
//      System.out.println(floatingJoint1.getQddy());
//      System.out.println(floatingJoint1.getQddz() + ", " + floatingJoint2.getQdd_t2());
//      
//      System.out.println(floatingJoint1.getAngularAccelerationX());
//      System.out.println(floatingJoint1.getAngularAccelerationY() + ", " + floatingJoint2.getQdd_rot());
//      System.out.println(floatingJoint1.getAngularAccelerationZ());

      double epsilon = 1e-9;
      assertEquals(floatingJoint1.getQddx().getDoubleValue(), floatingJoint2.getQdd_t1().getDoubleValue(), epsilon);
      assertEquals(floatingJoint1.getQddy().getDoubleValue(), 0.0, epsilon);
      assertEquals(floatingJoint1.getQddz().getDoubleValue(), floatingJoint2.getQdd_t2().getDoubleValue(), epsilon);

      assertEquals(floatingJoint1.getAngularAccelerationX().getDoubleValue(), 0.0, epsilon);
      assertEquals(floatingJoint1.getAngularAccelerationY().getDoubleValue(), floatingJoint2.getQdd_rot().getDoubleValue(), epsilon);
      assertEquals(floatingJoint1.getAngularAccelerationZ().getDoubleValue(), 0.0, epsilon);
   }

   private static double computeScalarInertiaAroundJointAxis(Link link, PinJoint pinJoint)
   {
      Matrix3d momentOfInertia = new Matrix3d();
      link.getMomentOfInertia(momentOfInertia);

      Vector3d jointAxis = new Vector3d();
      pinJoint.getJointAxis(jointAxis);
      Vector3d temp1 = new Vector3d(jointAxis);
      momentOfInertia.transform(temp1);
      double scalarInertiaAboutCoM = jointAxis.dot(temp1);    // jointAxis^T * momentOfInertia * jointAxis

      double mass = link.getMass();
      Vector3d offsetToCoM = new Vector3d(link.getComOffset());
      offsetToCoM.sub(pinJoint.getOffset());    // c - p
      Vector3d temp3 = new Vector3d(jointAxis);
      temp3.scale(offsetToCoM.dot(jointAxis));    // ((c - p) . a) * a
      Vector3d comToJointAxis = new Vector3d();
      comToJointAxis.sub(offsetToCoM, temp3);    // (c - p) - ((c - p) . a) * a
      double distanceToJointAxis = comToJointAxis.length();
      double scalarInertiaAboutJointAxis = scalarInertiaAboutCoM + mass * distanceToJointAxis * distanceToJointAxis;

      return scalarInertiaAboutJointAxis;
   }

   private void sleepIfShowingGUI() throws InterruptedException
   {
      if (SHOW_GUI)
      {
         while (true)
         {
            Thread.sleep(100L);
         }
      }
   }

   private static Link link11(Random random, double l1, double r1)
   {
      Link ret = new Link("link11");
      ret.setMass(random.nextDouble());

      ret.setComOffset(0.0, 0.0, -l1 / 2.0);
      ret.setMomentOfInertia(RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidEllipsoid(ret.getMass(), r1, r1, l1 / 2.0));

      LinkGraphics linkGraphics = new LinkGraphics();
      linkGraphics.addCoordinateSystem(COORDINATE_SYSTEM_LENGTH);
      linkGraphics.createInertiaEllipsoid(ret, YoAppearance.Red());
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private static Link link21(Random random, double l2, double r2)
   {
      Link ret = new Link("link2");
      ret.setMass(random.nextDouble());
      ret.setComOffset(0.0, 0.0, l2 / 2.0);

//    ret.setComOffset(0.0, 0.0, 0.0);
      ret.setMomentOfInertia(RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidEllipsoid(ret.getMass(), r2, r2, l2 / 2.0));

      LinkGraphics linkGraphics = new LinkGraphics();
      linkGraphics.addCoordinateSystem(COORDINATE_SYSTEM_LENGTH);
      linkGraphics.createInertiaEllipsoid(ret, YoAppearance.Orange());
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private static Link link22(Link link21)
   {
      Link ret = new Link("link22");
      ret.setComOffset(link21.getComOffset());
      ret.setMass(link21.getMass());
      Matrix3d link2moi = new Matrix3d();
      link21.getMomentOfInertia(link2moi);
      ret.setMomentOfInertia(link2moi);
      LinkGraphics linkGraphics = new LinkGraphics();
      linkGraphics.addCoordinateSystem(COORDINATE_SYSTEM_LENGTH);
      linkGraphics.createInertiaEllipsoid(ret, YoAppearance.Aqua());
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private static Link link12(Link link11, PinJoint pin1)
   {
      Link ret = new Link("link12");
      Vector3d comOffset12 = new Vector3d(link11.getComOffset());
      comOffset12.sub(pin1.getOffset());
      ret.setComOffset(comOffset12);
      ret.setMass(link11.getMass());
      Matrix3d link1moi = new Matrix3d();
      link11.getMomentOfInertia(link1moi);
      ret.setMomentOfInertia(link1moi);
      LinkGraphics linkGraphics = new LinkGraphics();
      linkGraphics.addCoordinateSystem(COORDINATE_SYSTEM_LENGTH);
      linkGraphics.createInertiaEllipsoid(ret, YoAppearance.Blue());
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private static Link randomBody(Random random)
   {
      Link ret = new Link("floatingBody");
      ret.setMass(random.nextDouble());
      ret.setComOffset(random.nextDouble(), random.nextDouble(), random.nextDouble());
      ret.setMomentOfInertia(RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidEllipsoid(ret.getMass(), random.nextDouble(), random.nextDouble(),
              random.nextDouble()));

      LinkGraphics linkGraphics = new LinkGraphics();
      linkGraphics.addCoordinateSystem(COORDINATE_SYSTEM_LENGTH);
      linkGraphics.createInertiaEllipsoid(ret, YoAppearance.Orange());
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private static Link randomBodyNoYCoMOffset(Random random)
   {
      Link ret = new Link("floatingBody");
      ret.setMass(random.nextDouble());
      ret.setComOffset(random.nextDouble(), 0.0, random.nextDouble());
      ret.setMomentOfInertia(RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidEllipsoid(ret.getMass(), random.nextDouble(), random.nextDouble(),
              random.nextDouble()));

      LinkGraphics linkGraphics = new LinkGraphics();
      linkGraphics.addCoordinateSystem(COORDINATE_SYSTEM_LENGTH);
      linkGraphics.createInertiaEllipsoid(ret, YoAppearance.Orange());
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }
   
   private Link massiveLink()
   {
      Link ret = new Link("massiveLink");
      ret.setMass(1e12);
      ret.setMomentOfInertia(1e8, 1e8, 1e8);

      return ret;
   }

   private static Point3d computeCoM(Robot robot)
   {
      Point3d com = new Point3d();
      robot.computeCenterOfMass(com);

      return com;
   }

   private static Vector3d computeLinearMomentum(Robot robot)
   {
      Vector3d linearMomentum = new Vector3d();
      robot.computeLinearMomentum(linearMomentum);

      return linearMomentum;
   }

   private static Vector3d computeAngularMomentum(Robot robot)
   {
      Vector3d angularMomentum = new Vector3d();
      robot.computeAngularMomentum(angularMomentum);

      return angularMomentum;
   }
}
