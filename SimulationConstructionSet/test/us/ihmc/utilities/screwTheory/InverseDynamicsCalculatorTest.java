package us.ihmc.utilities.screwTheory;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.Random;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RotationFunctions;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.FloatingJoint;
import com.yobotics.simulationconstructionset.Joint;
import com.yobotics.simulationconstructionset.Link;
import com.yobotics.simulationconstructionset.LinkGraphics;
import com.yobotics.simulationconstructionset.PinJoint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoAppearance;
import com.yobotics.simulationconstructionset.util.robotExplorer.RobotExplorer;

/**
 * This currently needs to be here because it uses SCS classes to test the inverse dynamics calculator, and SCS isn't on the IHMCUtilities build path
 * @author Twan Koolen
 *
 */
public class InverseDynamicsCalculatorTest
{
   private final Random random = new Random(100L);

   @Before
   public void setUp()
   {
   }

   @Test
   public void testChain()
   {
      Robot robot = new Robot("robot");
      robot.setGravity(0.0);

      ReferenceFrame inertialFrame = ReferenceFrame.constructAWorldFrame("inertial");
      ReferenceFrame worldFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("world", inertialFrame, new Transform3D());
      worldFrame.update();
      RigidBody world = new RigidBody("world", worldFrame);

      int nLinks = 4;
      ArrayList<PinJoint> revoluteJoints = new ArrayList<PinJoint>();
      ArrayList<RevoluteJoint> inverseDynamicsRevoluteJoints = new ArrayList<RevoluteJoint>();

      createRandomChainRobotAndSetJointPositionsAndVelocities(robot, revoluteJoints, inverseDynamicsRevoluteJoints, worldFrame, world, nLinks);

      InverseDynamicsCalculator calculator = new InverseDynamicsCalculator(inertialFrame, world);

      calculator.compute();

      setTorques(revoluteJoints, inverseDynamicsRevoluteJoints);

      createAndStartSimulation(robot);

      compareAccelerations(revoluteJoints, inverseDynamicsRevoluteJoints);
   }

   private void setTorques(ArrayList<PinJoint> revoluteJoints, ArrayList<RevoluteJoint> inverseDynamicsRevoluteJoints)
   {
      for (int i = 0; i < revoluteJoints.size(); i++)
      {
         revoluteJoints.get(i).setTau(inverseDynamicsRevoluteJoints.get(i).getTau());
      }
   }

   private void compareAccelerations(ArrayList<PinJoint> revoluteJoints, ArrayList<RevoluteJoint> inverseDynamicsRevoluteJoints)
   {
      double epsilon = 1e-3;    // hmm, needs to be pretty high...
      for (int i = 0; i < revoluteJoints.size(); i++)
      {
         PinJoint revoluteJoint = revoluteJoints.get(i);
         RevoluteJoint inverseDynamicsRevoluteJoint = inverseDynamicsRevoluteJoints.get(i);

         DoubleYoVariable qddVariable = revoluteJoint.getQDD();
         double qdd = qddVariable.getDoubleValue();
         double qddInverse = inverseDynamicsRevoluteJoint.getQdd();
         assertEquals(qddInverse, qdd, epsilon);
      }
   }

   private void createAndStartSimulation(Robot robot)
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, false);
      scs.disableGUIComponents();
      scs.setRecordDT(scs.getDT());
      Thread simThread = new Thread(scs, "InverseDynamicsCalculatorTest sim thread");
      simThread.start();
      scs.simulate(1);
      waitForSimulationToFinish(scs);
   }

   private void createRandomChainRobotAndSetJointPositionsAndVelocities(Robot robot, ArrayList<PinJoint> revoluteJointsToPack,
           ArrayList<RevoluteJoint> inverseDynamicsRevoluteJointsToPack, ReferenceFrame worldFrame, RigidBody world, int nLinks)
   {
//    FloatingJoint rootJoint = new FloatingJoint("root", new Vector3d(), robot);
//    robot.addRootJoint(rootJoint);
//    SixDoFJoint rootInverseDynamicsJoint = new SixDoFJoint("root", world, worldFrame);

      PinJoint rootJoint = new PinJoint("root", new Vector3d(), robot, 1);
      robot.addRootJoint(rootJoint);

      FrameVector rootJointAxis = new FrameVector(worldFrame);
      rootJoint.getJointAxis(rootJointAxis.getVector());
      RevoluteJoint rootInverseDynamicsJoint = new RevoluteJoint("root", world, worldFrame, rootJointAxis);

      setRandomPosVelAcc(rootJoint, rootInverseDynamicsJoint);

      revoluteJointsToPack.add(rootJoint);
      inverseDynamicsRevoluteJointsToPack.add(rootInverseDynamicsJoint);

      Joint currentJoint = rootJoint;
      InverseDynamicsJoint currentInverseDynamicsJoint = rootInverseDynamicsJoint;
      for (int i = 0; i < nLinks; i++)
      {
         Link link = new Link("link" + i);

         link.setMomentOfInertia(random.nextDouble(), random.nextDouble(), random.nextDouble());
         link.setMass(random.nextDouble());

//       link.setMomentOfInertia(2.0, 3.0, 4.0);
//       link.setMass(0.5);

         Vector3d comOffset = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());

//       Vector3d comOffset = new Vector3d(0.0, 0.0, 1.0);
         link.setComOffset(comOffset);

         LinkGraphics linkGraphics = new LinkGraphics();
         linkGraphics.createInertiaEllipsoid(link, YoAppearance.Red());
         link.setLinkGraphics(linkGraphics);
         currentJoint.setLink(link);

         Matrix3d momentOfInertia = new Matrix3d();
         link.getMomentOfInertia(momentOfInertia);
         ReferenceFrame nextFrame = createOffsetFrame(currentInverseDynamicsJoint, comOffset, "body" + i);
         nextFrame.update();
         GeneralizedInertia inertia = new GeneralizedInertia(nextFrame, momentOfInertia, link.getMass());
         RigidBody rigidBody = new RigidBody("body" + i, inertia, currentInverseDynamicsJoint);

         if (i < nLinks - 1)
         {
            int jointAxisNumber = i % 3;

//          int jointAxisNumber = 1;

//          Vector3d offset = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
            Vector3d linkOffset = new Vector3d(0.0, 0.0, 2.0);
            PinJoint nextJoint = new PinJoint("joint" + i, linkOffset, robot, jointAxisNumber);
            currentJoint.addJoint(nextJoint);

            String frameName = "beforeJoint" + i;
            ReferenceFrame beforeJointFrame = createOffsetFrame(currentInverseDynamicsJoint, linkOffset, frameName);
            beforeJointFrame.update();
            FrameVector jointAxis = new FrameVector(beforeJointFrame);
            nextJoint.getJointAxis(jointAxis.getVector());

            RevoluteJoint nextInverseDynamicsJoint = new RevoluteJoint("joint" + i, rigidBody, beforeJointFrame, jointAxis);

            setRandomPosVelAcc(nextJoint, nextInverseDynamicsJoint);

            revoluteJointsToPack.add(nextJoint);
            inverseDynamicsRevoluteJointsToPack.add(nextInverseDynamicsJoint);

            currentJoint = nextJoint;
            currentInverseDynamicsJoint = nextInverseDynamicsJoint;
         }
      }

      setRandomPosVelAcc(rootJoint, rootInverseDynamicsJoint);

      world.updateFramesRecursively();

//      RobotExplorer explorer = new RobotExplorer(robot);
//      StringBuffer buffer = new StringBuffer();
//      explorer.getRobotInformationAsStringBuffer(buffer);
//      System.out.print(buffer);
   }

   @SuppressWarnings("unused")
   private void setRandomPosVelAcc(FloatingJoint floatingJoint, SixDoFJoint sixDoFJoint)
   {
      Point3d rootPosition = new Point3d(random.nextDouble(), random.nextDouble(), random.nextDouble());

      double yaw = random.nextDouble();
      double pitch = random.nextDouble();
      double roll = random.nextDouble();

      Vector3d velocityInWorld = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Matrix3d worldToBody = new Matrix3d();
      RotationFunctions.setYawPitchRoll(worldToBody, yaw, pitch, roll);
      worldToBody.transpose();
      Vector3d velocityInBody = new Vector3d(velocityInWorld);
      worldToBody.transform(velocityInBody);

      Vector3d angularVelocityInBody = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());

      ReferenceFrame predecessorFrame = sixDoFJoint.getPredecessor().getBodyFixedFrame();
      ReferenceFrame successorFrame = sixDoFJoint.getSuccessor().getBodyFixedFrame();
      ReferenceFrame frameAfterJoint = sixDoFJoint.getFrameAfterJoint();

      Twist twist = new Twist(successorFrame, predecessorFrame, frameAfterJoint, velocityInBody, angularVelocityInBody);
      twist.changeFrame(successorFrame);

      Vector3d linearAcceleration = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Vector3d angularAcceleration = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
      SpatialAccelerationVector desiredAcceleration = new SpatialAccelerationVector(successorFrame, predecessorFrame, successorFrame, linearAcceleration,
                                                         angularAcceleration);

      floatingJoint.setXYZ(rootPosition, velocityInWorld);
      floatingJoint.setYawPitchRoll(yaw, pitch, roll, angularVelocityInBody.getX(), angularVelocityInBody.getY(), angularVelocityInBody.getZ());
      floatingJoint.setVelocity(velocityInWorld);

      sixDoFJoint.setPosition(rootPosition);
      sixDoFJoint.setRotation(yaw, pitch, roll);
      sixDoFJoint.setTwist(twist);
      sixDoFJoint.setAcceleration(desiredAcceleration);
   }

   private void setRandomPosVelAcc(PinJoint pinJoint, RevoluteJoint revoluteJoint)
   {
      double q = random.nextDouble();
      double qd = random.nextDouble();
      double desiredQdd = random.nextDouble();

//    double q = 1.0;
//    double qd = 1.0;
//    double desiredQdd = 1.0;


      pinJoint.setInitialState(q, qd);
      revoluteJoint.setQ(q);
      revoluteJoint.setQd(qd);
      revoluteJoint.setQdd(desiredQdd);
   }

   private static ReferenceFrame createOffsetFrame(InverseDynamicsJoint currentInverseDynamicsJoint, Vector3d offset, String frameName)
   {
      ReferenceFrame parentFrame = currentInverseDynamicsJoint.getFrameAfterJoint();
      Transform3D transformToParent = new Transform3D();
      transformToParent.set(offset);
      ReferenceFrame beforeJointFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(frameName, parentFrame, transformToParent);

      return beforeJointFrame;
   }

   private void waitForSimulationToFinish(SimulationConstructionSet scs)
   {
      while (scs.isRunning())
      {
         try
         {
            Thread.sleep(10);
         }
         catch (InterruptedException e)
         {
            e.printStackTrace();
         }
      }

      scs = null;
   }
}
