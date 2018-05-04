package us.ihmc.simulationconstructionset;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.Random;

import org.junit.AfterClass;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.commons.thread.ThreadTools;

public class RobotTest
{
   private static final double COORDINATE_SYSTEM_LENGTH = 0.3;
   private static final boolean SHOW_GUI = false;

   @AfterClass
   public static void finishedAllTestsMessage()
   {
      System.out.println("Finished RobotTest, moving on.");
      System.out.flush();
      System.err.flush();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.8)
	@Test(timeout=300000)
   public void testSwitchingRootJoint() throws InterruptedException, UnreasonableAccelerationException, SimulationExceededMaximumTimeException, ControllerFailureException
   {
      Random random = new Random(1765L);
      double l1 = 2.0, l2 = 2.0, r1 = 0.1, r2 = 0.05;


      Robot robot1 = new Robot("r1");
      FloatingJoint root1 = new FloatingJoint("root1", new Vector3D(), robot1);
      robot1.addRootJoint(root1);
      Link link11 = link11(random, l1, r1);
      root1.setLink(link11);

      Vector3D offset = new Vector3D(0.0, 0.0, l1);

//    Vector3d offset = new Vector3d();
      PinJoint pin1 = new PinJoint("pin1", offset, robot1, Axis.Y);
      root1.addJoint(pin1);
      Link link21 = link21(random, l2, r2);
      pin1.setLink(link21);

      Robot robot2 = new Robot("r2");
      FloatingJoint root2 = new FloatingJoint("root2", new Vector3D(), robot2);
      robot2.addRootJoint(root2);
      Link link22 = link22(link21);
      root2.setLink(link22);
      Vector3D jointAxis = new Vector3D();
      pin1.getJointAxis(jointAxis);
      PinJoint pin2 = new PinJoint("pin2", new Vector3D(), robot2, jointAxis);
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
      pin2.setQ(-pin1.getQYoVariable().getDoubleValue());

      pin1.setQd(10.0);
      pin2.setQd(-pin1.getQDYoVariable().getDoubleValue());
      Vector3D angularVelocityInBody = new Vector3D(0.0, pin1.getQDYoVariable().getDoubleValue(), 0.0);
      root2.setAngularVelocityInBody(angularVelocityInBody);

      pin1.setTau(random.nextDouble());
      pin2.setTau(-pin1.getTauYoVariable().getDoubleValue());

      robot1.update();
      RigidBodyTransform pin1ToWorld = new RigidBodyTransform();
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
      EuclidCoreTestTools.assertTuple3DEquals(computeCoM(robot1), computeCoM(robot2), epsilonBefore);
      EuclidCoreTestTools.assertTuple3DEquals(computeLinearMomentum(robot1), computeLinearMomentum(robot2), epsilonBefore);
      EuclidCoreTestTools.assertTuple3DEquals(computeAngularMomentum(robot1), computeAngularMomentum(robot2), epsilonBefore);
      assertEquals(computeScalarInertiaAroundJointAxis(link11, pin1), computeScalarInertiaAroundJointAxis(link12, pin2), epsilonBefore);
      assertEquals(computeScalarInertiaAroundJointAxis(link21, pin1), computeScalarInertiaAroundJointAxis(link22, pin2), epsilonBefore);

      robot1.doDynamicsButDoNotIntegrate();
      robot2.doDynamicsButDoNotIntegrate();

      assertEquals(pin1.getQDDYoVariable().getDoubleValue(), pin1.getQDDYoVariable().getDoubleValue(), 1e-8);

      double simulateTime = 1.0;
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(SHOW_GUI);
      
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot[] {robot1, robot2}, parameters);
      scs.setDT(1e-4, 10);
      scs.setGroundVisible(false);
      Thread simThread = new Thread(scs, "sim thread");
      simThread.start();
      BlockingSimulationRunner simulationRunner = new BlockingSimulationRunner(scs, 50.0);
      simulationRunner.simulateAndBlock(simulateTime);
      sleepIfShowingGUI();

      double epsilonAfter = 1e-4;
      EuclidCoreTestTools.assertTuple3DEquals(computeCoM(robot1), computeCoM(robot2), epsilonAfter);
      EuclidCoreTestTools.assertTuple3DEquals(computeLinearMomentum(robot1), computeLinearMomentum(robot2), epsilonAfter);
      EuclidCoreTestTools.assertTuple3DEquals(computeAngularMomentum(robot1), computeAngularMomentum(robot2), epsilonAfter);
      assertEquals(computeScalarInertiaAroundJointAxis(link11, pin1), computeScalarInertiaAroundJointAxis(link12, pin2), epsilonAfter);
      assertEquals(computeScalarInertiaAroundJointAxis(link21, pin1), computeScalarInertiaAroundJointAxis(link22, pin2), epsilonAfter);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout=300000)
   public void testSingleFloatingBodyWithCoMOffset() throws SimulationExceededMaximumTimeException, InterruptedException, UnreasonableAccelerationException, ControllerFailureException
   {
      Random random = new Random(1659L);
      Robot robot = new Robot("r1");
      robot.setGravity(0.0);

      FloatingJoint root1 = new FloatingJoint("root1", new Vector3D(), robot);
      robot.addRootJoint(root1);

      Link floatingBody = randomBody(random);
      root1.setLink(floatingBody);
      Vector3D comOffset = floatingBody.getComOffset();

      Vector3D externalForcePointOffset = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      ExternalForcePoint externalForcePoint = new ExternalForcePoint("efp", externalForcePointOffset, robot.getRobotsYoVariableRegistry());
      root1.addExternalForcePoint(externalForcePoint);
      Vector3D force = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      externalForcePoint.setForce(force);

      robot.doDynamicsButDoNotIntegrate();

      // EULER
      // moment about CoM
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      root1.getTransformToWorld(transformToWorld);
      RigidBodyTransform transformToBody = new RigidBodyTransform();
      transformToBody.setAndInvert(transformToWorld);
      transformToBody.transform(force);
      Vector3D moment = new Vector3D();
      moment.sub(externalForcePointOffset, comOffset);
      moment.cross(moment, force);

      // moment from dynamics
      Matrix3D momentOfInertia = new Matrix3D();
      floatingBody.getMomentOfInertia(momentOfInertia);

      Vector3D temp1 = new Vector3D();
      temp1.set(root1.getAngularAccelerationInBody());
      momentOfInertia.transform(temp1);    // J omegad

      Vector3D temp2 = new Vector3D();
      temp2.set(root1.getAngularVelocityInBody());
      momentOfInertia.transform(temp2);    // J omega
      temp2.cross(root1.getAngularVelocityInBody(), temp2);    // omega x J omega

      Vector3D angularMomentumDerivative = new Vector3D();
      angularMomentumDerivative.add(temp1, temp2);    // J omegad + omega x J omega
      EuclidCoreTestTools.assertTuple3DEquals(angularMomentumDerivative, moment, 1e-10);

      // NEWTON
      Vector3D linearMomentum0 = computeLinearMomentum(robot);

      double dt = 1e-8;
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(SHOW_GUI);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.setDT(1e-10, 10);
      scs.setGroundVisible(false);
      Thread simThread = new Thread(scs, "sim thread");
      simThread.start();
      BlockingSimulationRunner simulationRunner = new BlockingSimulationRunner(scs, 50.0);
      simulationRunner.simulateAndBlock(dt);

      Vector3D linearMomentumDT = computeLinearMomentum(robot);
      Vector3D linearMomentumDerivativeNumerical = new Vector3D();
      linearMomentumDerivativeNumerical.sub(linearMomentumDT, linearMomentum0);
      linearMomentumDerivativeNumerical.scale(1.0 / dt);
      EuclidCoreTestTools.assertTuple3DEquals(linearMomentumDerivativeNumerical, force, 1e-10);

//    sleepIfShowingGUI();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testFloatingJointAndPinJointWithMassiveBody() throws UnreasonableAccelerationException
   {
      Random random = new Random(1659L);
      Robot robot = new Robot("r1");
      robot.setGravity(0.0);

      FloatingJoint root1 = new FloatingJoint("root1", new Vector3D(), robot);
      robot.addRootJoint(root1);

      Link floatingBody = new Link("floatingBody");
      floatingBody.setMass(random.nextDouble());

      floatingBody.setComOffset(random.nextDouble(), random.nextDouble(), random.nextDouble());
      floatingBody.setMomentOfInertia(RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidEllipsoid(floatingBody.getMass(), random.nextDouble(),
              random.nextDouble(), random.nextDouble()));
      root1.setLink(floatingBody);

      Vector3D offset = RandomGeometry.nextVector3D(random);
      PinJoint pin1 = new PinJoint("pin1", offset, robot, RandomGeometry.nextVector3D(random));
      pin1.setLink(massiveLink());
      root1.addJoint(pin1);

      pin1.setTau(random.nextDouble());
      robot.doDynamicsButDoNotIntegrate();

      double scalarInertiaAboutJointAxis = computeScalarInertiaAroundJointAxis(floatingBody, pin1);

      double torqueFromDynamics = scalarInertiaAboutJointAxis * pin1.getQDDYoVariable().getDoubleValue();
      assertEquals(pin1.getTauYoVariable().getDoubleValue(), torqueFromDynamics, pin1.getTauYoVariable().getDoubleValue() * 1e-3);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testCalculateAngularMomentum()
   {
	   double epsilon = 1e-7;
	   Robot robot1 = new Robot("r1");
	   FloatingJoint floatingJoint1 = new FloatingJoint("joint1", new Vector3D(),robot1);
	   robot1.addRootJoint(floatingJoint1);
	   Link onlyLink=new Link("SphericalLink");
	   onlyLink.setComOffset(new Vector3D(0.0, 0.0, 0.0));
	   onlyLink.setMass(1.0);
	   onlyLink.setMomentOfInertia(1.0, 1.0, 1.0);
	   floatingJoint1.setLink(onlyLink);
	   floatingJoint1.setPosition(new Point3D(1.0,1.0,1.0));
	   floatingJoint1.setAngularVelocityInBody(new Vector3D(1.0,0.0,0.0));
	   floatingJoint1.setVelocity(-1.0, 0.0, 0.0);
	   Vector3D angularMomentumReturned = new Vector3D();
	   robot1.computeAngularMomentum(angularMomentumReturned);
	   EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(0.0, 0.0, 0.0), angularMomentumReturned, epsilon);
	   robot1.update();
	   robot1.computeAngularMomentum(angularMomentumReturned);
	   EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(0.0, 0.0, 0.0), angularMomentumReturned, epsilon);
	   robot1.updateVelocities();
	   robot1.computeAngularMomentum(angularMomentumReturned);
	   //System.out.printf("Momentum <%+3.4f,%+3.4f,%+3.4f>", angularMomentumReturned.x,angularMomentumReturned.y,angularMomentumReturned.z);
	   EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(1.0, -1.0, 1.0), angularMomentumReturned, epsilon);
	   floatingJoint1.setPosition(new Vector3D());
	   
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testCompareFloatingJointAndFLoatingPlanarJoint()
           throws UnreasonableAccelerationException, SimulationExceededMaximumTimeException, InterruptedException
   {
      long seed = 101L;
      Random random = new Random(seed);
      double gravity = 0.0;    // random.nextDouble();

      Vector3D offset = new Vector3D(random.nextDouble(), 0.0, random.nextDouble());

      Robot robot1 = new Robot("r1");
      FloatingJoint floatingJoint1 = new FloatingJoint("joint1", new Vector3D(), robot1);
      robot1.addRootJoint(floatingJoint1);
      floatingJoint1.setLink(randomBodyNoYCoMOffset(random));
      PinJoint pin1 = new PinJoint("pin1", offset, robot1, Axis.Y);
      floatingJoint1.addJoint(pin1);
      pin1.setLink(randomBodyNoYCoMOffset(random));

      Robot robot2 = new Robot("r2");
      FloatingPlanarJoint floatingJoint2 = new FloatingPlanarJoint("joint2", robot2);
      robot2.addRootJoint(floatingJoint2);
      floatingJoint2.setLink(new Link(floatingJoint1.getLink()));
      PinJoint pin2 = new PinJoint("pin2", offset, robot2, Axis.Y);
      floatingJoint2.addJoint(pin2);
      pin2.setLink(new Link(pin1.getLink()));

      Robot[] robots = new Robot[] {robot1, robot2};
      OneDegreeOfFreedomJoint[] pinJoints = new OneDegreeOfFreedomJoint[] {pin1, pin2};
      for (Robot robot : robots)
      {
//       System.out.println(new RobotExplorer(robot));
         robot.setGravity(gravity);
      }

      double q = random.nextDouble();
      double tau = random.nextDouble();
      for (OneDegreeOfFreedomJoint pinJoint : pinJoints)
      {
         pinJoint.setQ(q);
         pinJoint.setTau(tau);
      }

      for (Robot robot : robots)
      {
         robot.doDynamicsButDoNotIntegrate();
      }

//    double simulateTime = 2.0;
//    SimulationConstructionSet scs = new SimulationConstructionSet(robot1, SHOW_GUI);
//    scs.setDT(1e-4, 10);
//    scs.setGroundVisible(false);
//    Thread simThread = new Thread(scs, "sim thread");
//    simThread.start();
//    BlockingSimulationRunner simulationRunner = new BlockingSimulationRunner(scs, 50.0);
//    simulationRunner.simulateAndBlock(simulateTime);
//    sleepIfShowingGUI();

//    System.out.println(floatingJoint1.getQddx() + ", " + floatingJoint2.getQdd_t1());
//    System.out.println(floatingJoint1.getQddy());
//    System.out.println(floatingJoint1.getQddz() + ", " + floatingJoint2.getQdd_t2());
//    
//    System.out.println(floatingJoint1.getAngularAccelerationX());
//    System.out.println(floatingJoint1.getAngularAccelerationY() + ", " + floatingJoint2.getQdd_rot());
//    System.out.println(floatingJoint1.getAngularAccelerationZ());

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
      Matrix3D momentOfInertia = new Matrix3D();
      link.getMomentOfInertia(momentOfInertia);

      Vector3D jointAxis = new Vector3D();
      pinJoint.getJointAxis(jointAxis);
      Vector3D temp1 = new Vector3D(jointAxis);
      momentOfInertia.transform(temp1);
      double scalarInertiaAboutCoM = jointAxis.dot(temp1);    // jointAxis^T * momentOfInertia * jointAxis

      double mass = link.getMass();
      Vector3D offsetToCoM = new Vector3D(link.getComOffset());
      
      Vector3D offset = new Vector3D();
      pinJoint.getOffset(offset);
      offsetToCoM.sub(offset);    // c - p
      Vector3D temp3 = new Vector3D(jointAxis);
      temp3.scale(offsetToCoM.dot(jointAxis));    // ((c - p) . a) * a
      Vector3D comToJointAxis = new Vector3D();
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

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(COORDINATE_SYSTEM_LENGTH);
      createInertiaEllipsoid(ret, linkGraphics, YoAppearance.Red());
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   public static void createInertiaEllipsoid(Link ret, Graphics3DObject linkGraphics, AppearanceDefinition appearance)
   {
      Matrix3D momentOfInertia = new Matrix3D();
      ret.getMomentOfInertia(momentOfInertia);
      Vector3D comOffset = new Vector3D();
      ret.getComOffset(comOffset);
      double mass = ret.getMass();
      
      linkGraphics.createInertiaEllipsoid(momentOfInertia, comOffset, mass, appearance);
   }
   
   private static Link link21(Random random, double l2, double r2)
   {
      Link ret = new Link("link2");
      ret.setMass(random.nextDouble());
      ret.setComOffset(0.0, 0.0, l2 / 2.0);

//    ret.setComOffset(0.0, 0.0, 0.0);
      ret.setMomentOfInertia(RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidEllipsoid(ret.getMass(), r2, r2, l2 / 2.0));

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(COORDINATE_SYSTEM_LENGTH);
      createInertiaEllipsoid(ret, linkGraphics, YoAppearance.Orange());
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private static Link link22(Link link21)
   {
      Link ret = new Link("link22");
      ret.setComOffset(link21.getComOffset());
      ret.setMass(link21.getMass());
      Matrix3D link2moi = new Matrix3D();
      link21.getMomentOfInertia(link2moi);
      ret.setMomentOfInertia(link2moi);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(COORDINATE_SYSTEM_LENGTH);
      createInertiaEllipsoid(ret, linkGraphics, YoAppearance.Aqua());
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private static Link link12(Link link11, PinJoint pin1)
   {
      Link ret = new Link("link12");
      Vector3D comOffset12 = new Vector3D(link11.getComOffset());
      
      Vector3D offset = new Vector3D();
      pin1.getOffset(offset);
      comOffset12.sub(offset);
      ret.setComOffset(comOffset12);
      ret.setMass(link11.getMass());
      Matrix3D link1moi = new Matrix3D();
      link11.getMomentOfInertia(link1moi);
      ret.setMomentOfInertia(link1moi);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(COORDINATE_SYSTEM_LENGTH);
      createInertiaEllipsoid(ret, linkGraphics, YoAppearance.Blue());
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

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(COORDINATE_SYSTEM_LENGTH);
      createInertiaEllipsoid(ret, linkGraphics, YoAppearance.Orange());
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

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(COORDINATE_SYSTEM_LENGTH);
      createInertiaEllipsoid(ret, linkGraphics, YoAppearance.Orange());
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

   private static Point3D computeCoM(Robot robot)
   {
      Point3D com = new Point3D();
      robot.computeCenterOfMass(com);

      return com;
   }

   private static Vector3D computeLinearMomentum(Robot robot)
   {
      Vector3D linearMomentum = new Vector3D();
      robot.computeLinearMomentum(linearMomentum);

      return linearMomentum;
   }

   private static Vector3D computeAngularMomentum(Robot robot)
   {
      Vector3D angularMomentum = new Vector3D();
      robot.computeAngularMomentum(angularMomentum);

      return angularMomentum;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout=300000)
   public void testFreezeJointAtZero() throws UnreasonableAccelerationException
   {
      Robot robot = new Robot("robot");
      
      Vector3D jointOffset1 = new Vector3D(0.12, 1.17, 3.125);
      double mass1 = 1.12;
      Vector3D comOffset1 = new Vector3D(0.1, 1.11, 3.79);
      
      Matrix3D momentOfInertia1 = new Matrix3D();
      momentOfInertia1.setM00(1.95);
      momentOfInertia1.setM11(3.93);
      momentOfInertia1.setM22(7.91);
      
      Vector3D jointOffset2 = new Vector3D(-0.4, 1.76, 1.1);
      double mass2 = 1.12;
      Vector3D comOffset2 = new Vector3D();
      Matrix3D momentOfInertia2 = new Matrix3D();
      momentOfInertia1.setM00(51.95);
      momentOfInertia1.setM11(0.93);
      momentOfInertia1.setM22(7.51);
      
      SliderJoint joint1 = new SliderJoint("joint1", jointOffset1, robot, Axis.X);
      Link link1 = new Link("link1");
      link1.setMass(mass1);
      link1.setMomentOfInertia(momentOfInertia1);
      link1.setComOffset(comOffset1);
      joint1.setLink(link1);
      
      PinJoint joint2 = new PinJoint("joint2", jointOffset2, robot, Axis.X);
      Link link2 = new Link("link2");
      link2.setMass(mass2);
      link2.setMomentOfInertia(momentOfInertia2);
      link2.setComOffset(comOffset2);
      joint2.setLink(link2);
      
      robot.addRootJoint(joint1);
      joint1.addJoint(joint2);
      
      robot.freezeJointAtZero(joint2);
      
      Point3D comPoint = new Point3D();
      double totalMass = robot.computeCenterOfMass(comPoint);
      
      double epsilon = 1e-7;
      assertEquals(mass1+mass2, totalMass, epsilon);
      
      Vector3D jointOffset = new Vector3D();
      joint1.getOffset(jointOffset); 
      EuclidCoreTestTools.assertTuple3DEquals(jointOffset1, jointOffset, epsilon);
      
      Link link = joint1.getLink();
      Vector3D comOffset = new Vector3D();
      link.getComOffset(comOffset);
      
      Vector3D temp = new Vector3D(comOffset1);
      temp.scale(mass1);
      Vector3D expectedCoMOffset = new Vector3D(temp);
      temp.set(comOffset2);
      temp.add(jointOffset2);
      temp.scale(mass2);
      expectedCoMOffset.add(temp);
      expectedCoMOffset.scale(1.0/(mass1 + mass2));
      EuclidCoreTestTools.assertTuple3DEquals(expectedCoMOffset, comOffset, epsilon);
      
      Matrix3D momentOfInertia = new Matrix3D();
      link.getMomentOfInertia(momentOfInertia);
      
      Matrix3D expectedMomentOfInertia = new Matrix3D();
     
      Vector3D deltaR1 = new Vector3D();
      deltaR1.sub(comOffset, comOffset1);
      Matrix3D tempMatrix = new Matrix3D();

      parallelAxisTheorem(momentOfInertia1, mass1, deltaR1, tempMatrix);
      expectedMomentOfInertia.set(tempMatrix);
      
      Vector3D deltaR2 = new Vector3D();
      deltaR2.sub(comOffset, comOffset2);
      deltaR2.sub(jointOffset2);
      parallelAxisTheorem(momentOfInertia2, mass2, deltaR2, tempMatrix);

      expectedMomentOfInertia.add(tempMatrix);
      
      EuclidCoreTestTools.assertMatrix3DEquals("", expectedMomentOfInertia, momentOfInertia, epsilon);

      
      Robot expectedRobot = new Robot("expectedRobot");
      SliderJoint expectedJoint = new SliderJoint("expected", jointOffset, expectedRobot, Axis.X);
      Link expectedLink = new Link("expectedLink");
      expectedLink.setMass(totalMass);
      expectedLink.setComOffset(expectedCoMOffset);
      expectedLink.setMomentOfInertia(expectedMomentOfInertia);
      expectedJoint.setLink(expectedLink);
      expectedRobot.addRootJoint(expectedJoint);
      
      double qInitial = 0.35;
      double qdInitial = -1.11;
      expectedJoint.setInitialState(qInitial, qdInitial);
      joint1.setInitialState(qInitial, qdInitial);
      
      double dt = 0.0001;
      for (int i=0; i<1000; i++)
      {
         robot.doDynamicsAndIntegrate(dt);
         expectedRobot.doDynamicsAndIntegrate(dt);
      }
      
      assertFalse(Double.isNaN(expectedJoint.getQYoVariable().getDoubleValue()));

      assertEquals(expectedJoint.getQYoVariable().getDoubleValue(), joint1.getQYoVariable().getDoubleValue(), epsilon);
      assertEquals(expectedJoint.getQDYoVariable().getDoubleValue(), joint1.getQDYoVariable().getDoubleValue(), epsilon);
      assertEquals(expectedJoint.getQDDYoVariable().getDoubleValue(), joint1.getQDDYoVariable().getDoubleValue(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testFreezeJointAtZeroTwo() throws UnreasonableAccelerationException
   {
      Robot robotOne = createTestRobot();
      Robot robotTwo = createTestRobot();
      
      FloatingJoint rootJointOne = (FloatingJoint) robotOne.getRootJoints().get(0);
      ArrayList<Joint> childrenJointsOne = new ArrayList<Joint>();
      rootJointOne.getChildrenJoints(childrenJointsOne);
      robotOne.freezeJointAtZero(childrenJointsOne.get(0));
      robotOne.freezeJointAtZero(childrenJointsOne.get(1));
      
      FloatingJoint rootJointTwo = (FloatingJoint) robotTwo.getRootJoints().get(0);
      ArrayList<Joint> childrenJointsTwo = new ArrayList<Joint>();
      rootJointTwo.getChildrenJoints(childrenJointsTwo);
      robotOne.freezeJointAtZero(childrenJointsTwo.get(0));
      robotOne.freezeJointAtZero(childrenJointsTwo.get(1));

      Vector3D positionOne = new Vector3D(3.4, 5.7, 2.2);
      Vector3D velocityOne = new Vector3D(12.4, 15.9, 0.2);
      
      rootJointOne.setPosition(positionOne);
      rootJointOne.setVelocity(velocityOne);
      
      Vector3D positionTwo = new Vector3D(positionOne);
      Vector3D velocityTwo = new Vector3D(velocityOne);
      
      rootJointTwo.setPosition(positionTwo);
      rootJointTwo.setVelocity(velocityTwo);
      
      double dt = 0.0001;
      double epsilon = 1e-7;
      for (int i=0; i<1000; i++)
      {
         robotOne.doDynamicsAndIntegrate(dt);
         robotTwo.doDynamicsAndIntegrate(dt);
      }

      rootJointOne.getPositionAndVelocity(positionOne, velocityOne);
      rootJointTwo.getPositionAndVelocity(positionTwo, velocityTwo);
      
      assertFalse(Double.isNaN(positionOne.getX()));

      EuclidCoreTestTools.assertTuple3DEquals(positionOne, positionTwo, epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(velocityOne, velocityTwo, epsilon);
   }

   private Robot createTestRobot()
   {
      Robot robot = new Robot("robot");
      
      Vector3D rootOffset = new Vector3D(5.12, 6.17, 7.125);
      double rootMass = 10.12;
      Vector3D rootCoMOffset = new Vector3D(2.1, -1.11, 3.72);
      Matrix3D rootMomentOfInertia = new Matrix3D();
      rootMomentOfInertia.setM00(71.95);
      rootMomentOfInertia.setM11(83.93);
      rootMomentOfInertia.setM22(97.91);
      
      FloatingJoint rootJoint = new FloatingJoint("root", rootOffset, robot);
      Link rootLink = new Link("rootLink");
      rootLink.setMass(rootMass);
      rootLink.setComOffset(rootCoMOffset);
      rootLink.setMomentOfInertia(rootMomentOfInertia);
      
      rootJoint.setLink(rootLink);
      robot.addRootJoint(rootJoint);
      
      Vector3D jointOffset1 = new Vector3D(0.12, 1.17, 3.125);
      double mass1 = 1.12;
      Vector3D comOffset1 = new Vector3D(0.1, 1.11, 3.79);
      
      Matrix3D momentOfInertia1 = new Matrix3D();
      momentOfInertia1.setM00(1.95);
      momentOfInertia1.setM11(3.93);
      momentOfInertia1.setM22(7.91);
      
      Vector3D jointOffset2 = new Vector3D(-0.4, 1.76, 1.1);
      double mass2 = 1.12;
      Vector3D comOffset2 = new Vector3D();
      Matrix3D momentOfInertia2 = new Matrix3D();
      momentOfInertia1.setM00(51.95);
      momentOfInertia1.setM11(0.93);
      momentOfInertia1.setM22(7.51);
      
      PinJoint joint1 = new PinJoint("joint1", jointOffset1, robot, Axis.X);
      Link link1 = new Link("link1");
      link1.setMass(mass1);
      link1.setMomentOfInertia(momentOfInertia1);
      link1.setComOffset(comOffset1);
      joint1.setLink(link1);
      
      PinJoint joint2 = new PinJoint("joint2", jointOffset2, robot, Axis.X);
      Link link2 = new Link("link2");
      link2.setMass(mass2);
      link2.setMomentOfInertia(momentOfInertia2);
      link2.setComOffset(comOffset2);
      joint2.setLink(link2);
      
      rootJoint.addJoint(joint1);
      rootJoint.addJoint(joint2);
      
      return robot;
   }
   
   private void parallelAxisTheorem(Matrix3D inputInertia, double mass, Vector3D vector1, Matrix3D outputInertia)
   {
      outputInertia.set(inputInertia);
      
      double dotProduct = vector1.dot(vector1);

      for (int i=0; i<3; i++)
      {
         for (int j=0; j<3; j++)
         {
            double elementValueAdjustment = 0.0;
            if (i == j) elementValueAdjustment = dotProduct; 
            elementValueAdjustment = elementValueAdjustment - getElement(i, vector1) * getElement(j, vector1);
            elementValueAdjustment = elementValueAdjustment * mass;
            
            double elementValue = outputInertia.getElement(i, j);
            elementValue = elementValue + elementValueAdjustment;
            outputInertia.setElement(i, j, elementValue);
         }
      }
   }
   
   private double getElement(int index, Tuple3DBasics tuple)
   {
      if (index == 0) return tuple.getX();
      if (index == 1) return tuple.getY();
      if (index == 2) return tuple.getZ();
      throw new RuntimeException();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testChangeLinkParameters() throws UnreasonableAccelerationException
   {      
      Vector3D jointOffset1 = new Vector3D(0.12, 1.17, 3.125);
      double mass1 = 1.12;
      Vector3D comOffset1 = new Vector3D(0.1, 1.11, 3.79);
      
      Matrix3D momentOfInertia1 = new Matrix3D();
      momentOfInertia1.setM00(1.95);
      momentOfInertia1.setM11(3.93);
      momentOfInertia1.setM22(7.91);
      
      Vector3D jointOffset2 = new Vector3D(-0.4, 1.76, 1.1);
      double mass2 = 3.14;
      Vector3D comOffset2 = new Vector3D(0.1, 0.2, -1.79);
      Matrix3D momentOfInertia2 = new Matrix3D();
      momentOfInertia2.setM00(5.95);
      momentOfInertia2.setM11(0.93);
      momentOfInertia2.setM22(7.51);
      
      Vector3D jointOffset3 = new Vector3D(-0.33, 1.71, 2.1);
      double mass3 = 4.12;
      Vector3D comOffset3 = new Vector3D(0.2, 0.3, 12.79);
      Matrix3D momentOfInertia3 = new Matrix3D();
      momentOfInertia3.setM00(3.33);
      momentOfInertia3.setM11(7.7);
      momentOfInertia3.setM22(0.8);
      
      // Create the first robot
      Robot robotOne = new Robot("robotOne");

      PinJoint joint1 = new PinJoint("joint1", jointOffset1, robotOne, Axis.X);
      Link link1 = new Link("link1");
      link1.setMass(mass1);
      link1.setMomentOfInertia(momentOfInertia1);
      link1.setComOffset(comOffset1);
      joint1.setLink(link1);
      
      PinJoint joint2 = new PinJoint("joint2", jointOffset2, robotOne, Axis.Y);
      Link link2 = new Link("link2");
      link2.setMass(mass2);
      link2.setMomentOfInertia(momentOfInertia2);
      link2.setComOffset(comOffset2);
      joint2.setLink(link2);
      
      PinJoint joint3 = new PinJoint("joint3", jointOffset3, robotOne, Axis.Y);
      Link link3 = new Link("link3");
      link3.setMass(mass3);
      link3.setMomentOfInertia(momentOfInertia3);
      link3.setComOffset(comOffset3);
      joint3.setLink(link3);
      
      robotOne.addRootJoint(joint1);
      joint1.addJoint(joint2);
      joint2.addJoint(joint3);
      
      // Change the link stuff:
      jointOffset1.scale(0.77);
      joint1.changeOffsetVector(jointOffset1);
      
      mass1 = mass1 * 1.5;
      comOffset1.scale(0.33);
      momentOfInertia1.scale(0.789);

      link1.setMass(mass1);
      link1.setComOffset(comOffset1);
      link1.setMomentOfInertia(momentOfInertia1);
      
      jointOffset2.scale(3.33);
      joint2.changeOffsetVector(jointOffset2);
      
      mass2 = mass2 + 4.5;
      comOffset2.scale(3.91);
      momentOfInertia2.scale(3.33);
      
      Link newLink2 = new Link(joint2.getLink().getName());
      newLink2.setMass(mass2);
      newLink2.setComOffset(comOffset2);
      newLink2.setMomentOfInertia(momentOfInertia2);
      joint2.setLink(newLink2);
      
      jointOffset3.scale(0.33);
      joint3.changeOffsetVector(jointOffset3);
      
      mass3 = mass3 + 11.5;
      comOffset3.scale(2.33);
      momentOfInertia3.scale(4.789);

      link3.setMass(mass3);
      link3.setComOffset(comOffset3);
      link3.setMomentOfInertia(momentOfInertia3);
      
      // Make an identical robot straight up, without changing link stuff:
      Robot robotTwo = new Robot("robotTwo");

      PinJoint joint1B = new PinJoint("joint1", jointOffset1, robotTwo, Axis.X);
      Link link1B = new Link("link1");
      link1B.setMass(mass1);
      link1B.setMomentOfInertia(momentOfInertia1);
      link1B.setComOffset(comOffset1);
      joint1B.setLink(link1B);
      
      PinJoint joint2B = new PinJoint("joint2", jointOffset2, robotTwo, Axis.Y);
      Link link2B = new Link("link2");
      link2B.setMass(mass2);
      link2B.setMomentOfInertia(momentOfInertia2);
      link2B.setComOffset(comOffset2);
      joint2B.setLink(link2B);
      
      PinJoint joint3B = new PinJoint("joint3", jointOffset3, robotTwo, Axis.Y);
      Link link3B = new Link("link3");
      link3B.setMass(mass3);
      link3B.setMomentOfInertia(momentOfInertia3);
      link3B.setComOffset(comOffset3);
      joint3B.setLink(link3B);
      
      robotTwo.addRootJoint(joint1B);
      joint1B.addJoint(joint2B);
      joint2B.addJoint(joint3B);
      
      // Make sure the two robots are dynamically consistent:
      joint1.setInitialState(0.11, 0.33);
      joint2.setInitialState(0.15, 0.34);
      joint1.setTau(7.7);
      joint2.setTau(4.0);
      
      joint1B.setInitialState(0.11, 0.33);
      joint2B.setInitialState(0.15, 0.34);
      joint1B.setTau(7.7);
      joint2B.setTau(4.0);
      
      double DT = 0.00001;
      for (int i=0; i<100; i++)
      {
         robotOne.doDynamicsAndIntegrate(DT);
         robotTwo.doDynamicsAndIntegrate(DT);
      }
      
      double epsilon = 1e-7;
      
      assertFalse(Double.isNaN(joint1.getQYoVariable().getDoubleValue()));
      
      assertEquals(joint1.getQYoVariable().getDoubleValue(), joint1B.getQYoVariable().getDoubleValue(), epsilon);
      assertEquals(joint1.getQDYoVariable().getDoubleValue(), joint1B.getQDYoVariable().getDoubleValue(), epsilon);
      assertEquals(joint1.getQDDYoVariable().getDoubleValue(), joint1B.getQDDYoVariable().getDoubleValue(), epsilon);
      
      assertEquals(joint2.getQYoVariable().getDoubleValue(), joint2B.getQYoVariable().getDoubleValue(), epsilon);
      assertEquals(joint2.getQDYoVariable().getDoubleValue(), joint2B.getQDYoVariable().getDoubleValue(), epsilon);
      assertEquals(joint2.getQDDYoVariable().getDoubleValue(), joint2B.getQDDYoVariable().getDoubleValue(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.5)
	@Test(timeout=300000)
   public void testConservationOfEnergyAndMomentum() throws UnreasonableAccelerationException
   {
      String name = "robot";
      boolean startWithFloatingJoint = true;
      int numberOfPinJoints = 4;
      Random random = new Random(1776L);
      Robot robot = RandomRobotGenerator.generateRandomLinearChainRobot(name, startWithFloatingJoint, numberOfPinJoints, random);
      robot.setGravity(0.0, 0.0, 0.0);

      RandomRobotGenerator.setRandomJointPositions(robot, random);
      RandomRobotGenerator.setRandomJointVelocities(robot, random);

      double simulateDT = 0.00001;
      int numberOfTicksToSimulate = 8000;

      SimulationConstructionSet scs = null;
      if (SHOW_GUI)
      {
         scs = new SimulationConstructionSet(robot);
         int recordFrequency = 1;
         scs.setDT(simulateDT, recordFrequency);
         scs.startOnAThread();
      }
      
      Vector3D angularMomentumStart = new Vector3D();
      Vector3D linearMomentumStart = new Vector3D();

      robot.doDynamicsAndIntegrate(simulateDT);
      robot.update();
      
      robot.computeAngularMomentum(angularMomentumStart);
      robot.computeLinearMomentum(linearMomentumStart);
      double translationalKineticEnergyStart = robot.computeTranslationalKineticEnergy();
      double rotationalKineticEnergyStart = robot.computeRotationalKineticEnergy();
      double totalEnergyStart = translationalKineticEnergyStart + rotationalKineticEnergyStart;

      YoVariableRegistry registry = robot.getRobotsYoVariableRegistry();
      YoDouble translationalKineticEnergy = new YoDouble("translationalKineticEnergy", registry);
      YoDouble rotationalKineticEnergy = new YoDouble("rotationalKineticEnergy", registry);
      YoDouble totalEnergy = new YoDouble("totalEnergy", registry);
      YoFrameVector3D angularMomentum = new YoFrameVector3D("angularMomentum", ReferenceFrame.getWorldFrame(), registry);
      YoFrameVector3D linearMomentum = new YoFrameVector3D("linearMomentum", ReferenceFrame.getWorldFrame(), registry);

      Vector3D temp = new Vector3D();
      for (int i = 0; i < numberOfTicksToSimulate; i++)
      {
         translationalKineticEnergy.set(robot.computeTranslationalKineticEnergy());
         rotationalKineticEnergy.set(robot.computeRotationalKineticEnergy());
         totalEnergy.set(translationalKineticEnergy.getDoubleValue() + rotationalKineticEnergy.getDoubleValue());

         robot.computeAngularMomentum(temp);
         angularMomentum.set(temp);
         robot.computeLinearMomentum(temp);
         linearMomentum.set(temp);

         robot.updateVelocities();
         robot.doDynamicsAndIntegrate(simulateDT);
         robot.update();

         if (SHOW_GUI)
         {
            scs.tickAndUpdate();
         }
      }

      Vector3D angularMomentumEnd = new Vector3D();
      Vector3D linearMomentumEnd = new Vector3D();

      robot.computeAngularMomentum(angularMomentumEnd);
      robot.computeLinearMomentum(linearMomentumEnd);
      double translationalKineticEnergyEnd = robot.computeTranslationalKineticEnergy();
      double rotationalKineticEnergyEnd = robot.computeRotationalKineticEnergy();
      double totalEnergyEnd = translationalKineticEnergyEnd + rotationalKineticEnergyEnd;

      double epsilon = 1e-7;
      assertEquals("Total energy must be conserved", totalEnergyStart, totalEnergyEnd, epsilon);
      
      epsilon = 6e-5;
      EuclidCoreTestTools.assertTuple3DEquals("Angular momentum should be conserved", angularMomentumStart, angularMomentumEnd, epsilon);
      EuclidCoreTestTools.assertTuple3DEquals("Linear momentum should be conserved", linearMomentumStart, linearMomentumEnd, epsilon);
   
      if (SHOW_GUI) ThreadTools.sleepForever();
   }
}
