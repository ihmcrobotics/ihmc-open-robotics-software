package com.yobotics.simulationconstructionset;

import java.util.Random;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.utilities.math.RotationalInertiaCalculator;
import us.ihmc.utilities.test.JUnitTools;

import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class RobotTest
{
   private static final double COORDINATE_SYSTEM_LENGTH = 0.3;
   private static final boolean SHOW_GUI = false;

   // FIXME: test doesn't work properly yet!
   @Test
   public void testSwitchingRootJoint() throws InterruptedException, UnreasonableAccelerationException, SimulationExceededMaximumTimeException
   {
      Random random = new Random(1765L);
      double l1 = 1.0, l2 = 2.0, r1 = 0.1, r2 = 0.05;


      Robot robot1 = new Robot("r1");
      FloatingJoint root1 = new FloatingJoint("root1", new Vector3d(), robot1);
      robot1.addRootJoint(root1);
      Link link11 = link11(random, l1, r1);
      root1.setLink(link11);
//      Vector3d offset = new Vector3d(0.0, 0.0, L1);
      Vector3d offset = new Vector3d();
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


//      pin1.setQ(random.nextDouble());
//      pin2.setQ(-pin1.getQ().getDoubleValue());

      pin1.setQd(10.0);
      pin2.setQd(-pin1.getQD().getDoubleValue());
      Vector3d angularVelocityInBody = new Vector3d(0.0, pin1.getQD().getDoubleValue(), 0.0);
      root2.setAngularVelocityInBody(angularVelocityInBody);
//      Vector3d linearVelocityInBody = new Vector3d();
//      linearVelocityInBody.cross(pin1.getOffset(), angularVelocityInBody);
//      root2.setVelocity(linearVelocityInBody); // OK for now since 
//      root2.setVelocity(new Vector3d(0.0, L1 * pin1.getQD().getDoubleValue(), 0.0));
     
//      pin1.setTau(random.nextDouble());
//      pin2.setTau(-pin1.getTau().getDoubleValue());

      robot1.update();
      Transform3D pin1ToWorld = new Transform3D();
      pin1.getTransformToWorld(pin1ToWorld);
      root2.setRotationAndTranslation(pin1ToWorld);
      root2.setXYZ(root2.getQx().getDoubleValue(), root2.getQy().getDoubleValue(), root2.getQz().getDoubleValue());
      robot2.update();
      
      robot1.updateVelocities();
      robot2.updateVelocities();

      System.out.println("com1: " + computeCoM(robot1));
      System.out.println("com2: " + computeCoM(robot2));
      
      System.out.println("linearMomentum1: " + computeLinearMomentum(robot1));
      System.out.println("linearMomentum2: " + computeLinearMomentum(robot2));
      
      System.out.println("angularMomentum1: " + computeAngularMomentum(robot1));
      System.out.println("angularMomentum2: " + computeAngularMomentum(robot2));

      JUnitTools.assertTuple3dEquals(computeCoM(robot1), computeCoM(robot2), 1e-8);
      JUnitTools.assertTuple3dEquals(computeLinearMomentum(robot1), computeLinearMomentum(robot2), 1e-8);
      JUnitTools.assertTuple3dEquals(computeAngularMomentum(robot1), computeAngularMomentum(robot2), 1e-8);
      
      robot1.doDynamicsButDoNotIntegrate();
      robot2.doDynamicsButDoNotIntegrate();
      
      System.out.println(pin1.getQDD() + ", " + pin2.getQDD());
      System.out.println(root1.getAngularAccelerationY() + ", " + root2.getAngularAccelerationY());

      

      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot[] {robot1, robot2}, SHOW_GUI);
      scs.setDT(1e-4, 10);
      scs.setGroundVisible(false);
      Thread simThread = new Thread(scs, "sim thread");
      simThread.start();
      BlockingSimulationRunner simulationRunner = new BlockingSimulationRunner(scs, 50.0);
      double simulateTime = 1.0;
      simulationRunner.simulateAndBlock(simulateTime);

//      System.out.println(computeCoM(r1));
//      System.out.println(computeCoM(r2));

      if (SHOW_GUI)
      {         
         while (true)
         {
            Thread.sleep(100L);
         }
      }

      // FIXME: this currently fails!:
//    JUnitTools.assertTuple3dEquals(computeCoM(r1), computeCoM(r2), 1e-3);
   }

   private static Link link11(Random random, double l1, double r1)
   {
      Link ret = new Link("link11");
      ret.setMass(random.nextDouble());
//      ret.setComOffset(0.0, 0.0, L1 / 2.0);
      ret.setComOffset(0.0, 0.0, -l1 / 2.0);
//      ret.setComOffset(0.0, 0.0, 0.0);
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
//      ret.setComOffset(0.0, 0.0, 0.0);
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
