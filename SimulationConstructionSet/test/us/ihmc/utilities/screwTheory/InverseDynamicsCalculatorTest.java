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
import us.ihmc.utilities.test.JUnitTools;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.ExternalForcePoint;
import com.yobotics.simulationconstructionset.FloatingJoint;
import com.yobotics.simulationconstructionset.Joint;
import com.yobotics.simulationconstructionset.Link;
import com.yobotics.simulationconstructionset.LinkGraphics;
import com.yobotics.simulationconstructionset.PinJoint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoAppearance;

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
   public void testOneFreeRigidBody()
   {
      Robot robot = new Robot("robot");
      robot.setGravity(0.0);

      ReferenceFrame inertialFrame = ReferenceFrame.constructAWorldFrame("inertial");
      ReferenceFrame worldFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("world", inertialFrame, new Transform3D());
      RigidBody world = new RigidBody("world", worldFrame);

      FloatingJoint rootJoint = new FloatingJoint("root", new Vector3d(), robot);
      robot.addRootJoint(rootJoint);

      SixDoFJoint rootInverseDynamicsJoint = new SixDoFJoint("root", world, worldFrame);

      Link link = createRandomLink("link", true);
      rootJoint.setLink(link);
      Vector3d comOffset = new Vector3d();
      link.getComOffset(comOffset);

      RigidBody body = copyLinkAsRigidBody(link, rootInverseDynamicsJoint, "body");

      setRandomPosition(rootJoint, rootInverseDynamicsJoint);

      world.updateFramesRecursively();
      
      ReferenceFrame bodyFixedFrame = body.getBodyFixedFrame();
      ExternalForcePoint externalForcePoint = new ExternalForcePoint("rootExternalForcePoint", comOffset, robot);
      rootJoint.addExternalForcePoint(externalForcePoint);
      externalForcePoint.fx.set(random.nextDouble());
      externalForcePoint.fy.set(random.nextDouble());
      externalForcePoint.fz.set(random.nextDouble());

      Vector3d externalForce = new Vector3d();
      externalForcePoint.getForce(externalForce); // in world frame
      Transform3D worldToBody = worldFrame.getTransformToDesiredFrame(bodyFixedFrame);
      worldToBody.transform(externalForce);

      Wrench inputWrench = new Wrench(bodyFixedFrame, bodyFixedFrame, externalForce, new Vector3d());
      createAndStartSimulation(robot);

      copyAccelerationFromForwardToInverse(rootJoint, rootInverseDynamicsJoint);

      InverseDynamicsCalculator calculator = new InverseDynamicsCalculator(inertialFrame, world);
      calculator.compute();

      Wrench outputWrench = new Wrench(null, null);
      rootInverseDynamicsJoint.packWrench(outputWrench);

      compareWrenches(inputWrench, outputWrench);
   }

   private void compareWrenches(Wrench inputWrench, Wrench outputWrench)
   {
      inputWrench.getBodyFrame().checkReferenceFrameMatch(outputWrench.getBodyFrame());
      inputWrench.getExpressedInFrame().checkReferenceFrameMatch(outputWrench.getExpressedInFrame());

      double epsilon = 1e-3;
      JUnitTools.assertVector3dEquals(inputWrench.getTorque(), outputWrench.getTorque(), epsilon);
      JUnitTools.assertVector3dEquals(inputWrench.getForce(), outputWrench.getForce(), epsilon);
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
         String linkName = "link" + i;
         Link link = createRandomLink(linkName, false);
         currentJoint.setLink(link);

         String bodyName = "body" + i;
         RigidBody rigidBody = copyLinkAsRigidBody(link, currentInverseDynamicsJoint, bodyName);

         if (i < nLinks - 1)
         {
            String jointName = "joint" + i;

            int jointAxisNumber = i % 3;

            Vector3d linkOffset = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
            PinJoint nextJoint = new PinJoint(jointName, linkOffset, robot, jointAxisNumber);
            currentJoint.addJoint(nextJoint);

            String frameName = "beforeJoint" + i;
            ReferenceFrame beforeJointFrame = createOffsetFrame(currentInverseDynamicsJoint, linkOffset, frameName);
            beforeJointFrame.update();
            FrameVector jointAxis = new FrameVector(beforeJointFrame);
            nextJoint.getJointAxis(jointAxis.getVector());

            RevoluteJoint nextInverseDynamicsJoint = new RevoluteJoint(jointName, rigidBody, beforeJointFrame, jointAxis);

            setRandomPosVelAcc(nextJoint, nextInverseDynamicsJoint);

            revoluteJointsToPack.add(nextJoint);
            inverseDynamicsRevoluteJointsToPack.add(nextInverseDynamicsJoint);

            currentJoint = nextJoint;
            currentInverseDynamicsJoint = nextInverseDynamicsJoint;
         }
      }

      world.updateFramesRecursively();

//    RobotExplorer explorer = new RobotExplorer(robot);
//    StringBuffer buffer = new StringBuffer();
//    explorer.getRobotInformationAsStringBuffer(buffer);
//    System.out.print(buffer);
   }

   private Link createRandomLink(String linkName, boolean useZeroCoMOffset)
   {
      Link link = new Link(linkName);

      link.setMomentOfInertia(random.nextDouble(), random.nextDouble(), random.nextDouble());
      link.setMass(random.nextDouble());

      Vector3d comOffset = useZeroCoMOffset ? new Vector3d() : new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
      link.setComOffset(comOffset);

      LinkGraphics linkGraphics = new LinkGraphics();
      linkGraphics.createInertiaEllipsoid(link, YoAppearance.Red());
      link.setLinkGraphics(linkGraphics);

      return link;
   }

   private RigidBody copyLinkAsRigidBody(Link link, InverseDynamicsJoint currentInverseDynamicsJoint, String bodyName)
   {
      Vector3d comOffset = new Vector3d();
      link.getComOffset(comOffset);
      Matrix3d momentOfInertia = new Matrix3d();
      link.getMomentOfInertia(momentOfInertia);
      ReferenceFrame nextFrame = createOffsetFrame(currentInverseDynamicsJoint, comOffset, bodyName);
      nextFrame.update();
      GeneralizedInertia inertia = new GeneralizedInertia(nextFrame, momentOfInertia, link.getMass());
      RigidBody rigidBody = new RigidBody(bodyName, inertia, currentInverseDynamicsJoint);

      return rigidBody;
   }

   private void setRandomPosition(FloatingJoint floatingJoint, SixDoFJoint sixDoFJoint)
   {
      Point3d rootPosition = new Point3d(random.nextDouble(), random.nextDouble(), random.nextDouble());

      double yaw = random.nextDouble();
      double pitch = random.nextDouble();
      double roll = random.nextDouble();
      
      floatingJoint.setPosition(rootPosition);
      floatingJoint.setYawPitchRoll(yaw, pitch, roll);
      
      sixDoFJoint.setPosition(rootPosition);
      sixDoFJoint.setRotation(yaw, pitch, roll);
   }

   private void copyAccelerationFromForwardToInverse(FloatingJoint floatingJoint, SixDoFJoint sixDoFJoint) // THIS IS WHERE THE PROBLEM IS. ACCELERATIONS DON'T WORK THIS WAY
   {
      ReferenceFrame frameBeforeJoint = sixDoFJoint.getFrameBeforeJoint();
      ReferenceFrame frameAfterJoint = sixDoFJoint.getFrameAfterJoint();
      ReferenceFrame successorFrame = sixDoFJoint.getSuccessor().getBodyFixedFrame();

      double linearAccelerationX = floatingJoint.getQddx().getDoubleValue();
      double linearAccelerationY = floatingJoint.getQddy().getDoubleValue();
      double linearAccelerationZ = floatingJoint.getQddz().getDoubleValue();

      FrameVector linearAcceleration = new FrameVector(frameBeforeJoint, linearAccelerationX, linearAccelerationY, linearAccelerationZ);
      linearAcceleration = linearAcceleration.changeFrameCopy(frameAfterJoint);

      double angularAccelerationX = floatingJoint.getAngularAccelerationX().getDoubleValue();
      double angularAccelerationY = floatingJoint.getAngularAccelerationY().getDoubleValue();
      double angularAccelerationZ = floatingJoint.getAngularAccelerationZ().getDoubleValue();
      FrameVector angularAcceleration = new FrameVector(frameAfterJoint, angularAccelerationX, angularAccelerationY, angularAccelerationZ);

      SpatialAccelerationVector jointAcceleration = new SpatialAccelerationVector(successorFrame, frameBeforeJoint, successorFrame,
                                                       linearAcceleration.getVector(), angularAcceleration.getVector());

      sixDoFJoint.setAcceleration(jointAcceleration);
   }

   private void setRandomPosVelAcc(PinJoint pinJoint, RevoluteJoint revoluteJoint)
   {
      double q = random.nextDouble();
      double qd = random.nextDouble();
      double desiredQdd = random.nextDouble();
      
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
