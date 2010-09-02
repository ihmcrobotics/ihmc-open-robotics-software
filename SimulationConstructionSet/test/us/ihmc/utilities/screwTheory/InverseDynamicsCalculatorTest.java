package us.ihmc.utilities.screwTheory;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.Random;

import javax.media.j3d.Transform3D;
import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
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

      int nLinks = 5;
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
      double epsilon = 1e-8;
      for (int i = 0; i < revoluteJoints.size(); i++)
      {
         PinJoint revoluteJoint = revoluteJoints.get(i);
         RevoluteJoint inverseDynamicsRevoluteJoint = inverseDynamicsRevoluteJoints.get(i);

         DoubleYoVariable qddVariable = revoluteJoint.getQDD();
         double qdd = qddVariable.getDoubleValue();
         double qddInverse = inverseDynamicsRevoluteJoint.getQdd();
         assertEquals(qdd, qddInverse, epsilon);
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

      PinJoint rootJoint = new PinJoint("root", new Vector3d(), robot, 0);
      robot.addRootJoint(rootJoint);
      
      FrameVector rootJointAxis = new FrameVector(worldFrame);
      rootJoint.getJointAxis(rootJointAxis.getVector());
      RevoluteJoint rootInverseDynamicsJoint = new RevoluteJoint("root", world, worldFrame, rootJointAxis);

      setRandomPosVelAcc(rootJoint, rootInverseDynamicsJoint);

      revoluteJointsToPack.add(rootJoint);
      inverseDynamicsRevoluteJointsToPack.add(rootInverseDynamicsJoint);

      Joint currentJoint = rootJoint;
      InverseDynamicsJoint currentInverseDynamicsJoint = rootInverseDynamicsJoint;
      ReferenceFrame currentFrame = worldFrame;
      for (int i = 0; i < nLinks; i++)
      {
         Link link = new Link("link" + i);
         link.setMomentOfInertia(random.nextDouble(), random.nextDouble(), random.nextDouble());
         link.setMass(random.nextDouble());
         Vector3d comOffset = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
         link.setComOffset(comOffset);
         LinkGraphics linkGraphics = new LinkGraphics();
         linkGraphics.createInertiaEllipsoid(link, YoAppearance.Red());
         link.setLinkGraphics(linkGraphics);
         currentJoint.setLink(link);

         Matrix3d momentOfInertia = new Matrix3d();
         link.getMomentOfInertia(momentOfInertia);
         ReferenceFrame nextFrame = createCoMFrame(currentFrame, currentJoint, comOffset, "body" + i);
         nextFrame.update();
         GeneralizedInertia inertia = new GeneralizedInertia(nextFrame, momentOfInertia, link.getMass());
         RigidBody rigidBody = new RigidBody("body" + i, inertia, currentInverseDynamicsJoint);
         currentInverseDynamicsJoint.setSuccessor(rigidBody);

         if (i < nLinks - 1)
         {
            int jointAxisNumber = i % 3;
            Vector3d offset = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
            PinJoint nextJoint = new PinJoint("joint" + i, offset, robot, jointAxisNumber);
            currentJoint.addJoint(nextJoint);

            ReferenceFrame parentFrame = rigidBody.getParentJoint().getFrameAfterJoint();
            Transform3D transformToParent = new Transform3D();
            transformToParent.set(offset);
            ReferenceFrame beforeJointFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent("beforeJoint" + i, parentFrame,
                                                 transformToParent);
            beforeJointFrame.update();
            FrameVector jointAxis = new FrameVector(beforeJointFrame);
            currentJoint.getJointAxis(jointAxis.getVector());

            RevoluteJoint nextInverseDynamicsJoint = new RevoluteJoint("joint" + i, rigidBody, beforeJointFrame, jointAxis);

            setRandomPosVelAcc(nextJoint, nextInverseDynamicsJoint);

            revoluteJointsToPack.add(nextJoint);
            inverseDynamicsRevoluteJointsToPack.add(nextInverseDynamicsJoint);

            currentJoint = nextJoint;
            currentInverseDynamicsJoint = nextInverseDynamicsJoint;
            currentFrame = nextFrame;
         }
      }

      setRandomPosVelAcc(rootJoint, rootInverseDynamicsJoint);
      
      RobotExplorer explorer = new RobotExplorer(robot);
      StringBuffer buffer = new StringBuffer();
      explorer.getRobotInformationAsStringBuffer(buffer);
      System.out.print(buffer);
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

      pinJoint.setInitialState(q, qd);
      revoluteJoint.setQ(q);
      revoluteJoint.setQd(qd);
      revoluteJoint.setQdd(desiredQdd);
   }

   private ReferenceFrame createCoMFrame(ReferenceFrame parentFrame, Joint parentJoint, Vector3d comOffset, String name)
   {
      Vector3d offsetBeforeJoint = new Vector3d();
      parentJoint.getOffset(offsetBeforeJoint);

      if (parentJoint instanceof FloatingJoint)
      {
         FloatingJoint parentJointCast = (FloatingJoint) parentJoint;
         DoubleYoVariable qx = parentJointCast.getQx();
         DoubleYoVariable qy = parentJointCast.getQy();
         DoubleYoVariable qz = parentJointCast.getQz();

         DoubleYoVariable q_qs = parentJointCast.getQuaternionQs();
         DoubleYoVariable q_qx = parentJointCast.getQuaternionQx();
         DoubleYoVariable q_qy = parentJointCast.getQuaternionQy();
         DoubleYoVariable q_qz = parentJointCast.getQuaternionQz();

         return new FloatingJointCoMReferenceFrame(name, parentFrame, comOffset, qx, qy, qz, q_qs, q_qx, q_qy, q_qz);
      }
      else if (parentJoint instanceof PinJoint)
      {
         PinJoint parentJointCast = (PinJoint) parentJoint;
         DoubleYoVariable jointVariable = parentJointCast.getQ();
         Vector3d axis = new Vector3d();
         parentJoint.getJointAxis(axis);

         return new RevoluteJointCoMReferenceFrame(name, parentFrame, jointVariable, offsetBeforeJoint, comOffset, axis);
      }
      else
      {
         throw new RuntimeException("Unhandled joint type.");
      }
   }

   private static class RevoluteJointCoMReferenceFrame extends ReferenceFrame
   {
      private static final long serialVersionUID = 7391845311049967855L;
      private final DoubleYoVariable jointVariable;
      private final Vector3d offsetBeforeJoint;
      private final Vector3d offsetAfterJoint;
      private final Vector3d axis;

      public RevoluteJointCoMReferenceFrame(String frameName, ReferenceFrame parentFrame, DoubleYoVariable jointVariable, Vector3d offsetBeforeJoint,
              Vector3d offsetAfterJoint, Vector3d axis)
      {
         super(frameName, parentFrame);
         this.jointVariable = jointVariable;
         this.offsetBeforeJoint = new Vector3d(offsetBeforeJoint);
         this.offsetAfterJoint = new Vector3d(offsetAfterJoint);
         this.axis = new Vector3d(axis);
      }

      @Override
      public void updateTransformToParent(Transform3D transformToParent)
      {
         transformToParent.setTranslation(offsetBeforeJoint);

         Transform3D rotation = new Transform3D();
         AxisAngle4d axisAngle = new AxisAngle4d();
         double q = jointVariable.getDoubleValue();
         axisAngle.set(axis, q);
         rotation.set(axisAngle);
         transformToParent.mul(rotation);

         Transform3D translation = new Transform3D();
         translation.setTranslation(offsetAfterJoint);
         transformToParent.mul(translation);
      }
   }


   private static class FloatingJointCoMReferenceFrame extends ReferenceFrame
   {
      private static final long serialVersionUID = 2552863884666805932L;
      private final Vector3d comOffset;

      private final DoubleYoVariable qx, qy, qz;
      private final DoubleYoVariable q_qs, q_qx, q_qy, q_qz;

      public FloatingJointCoMReferenceFrame(String name, ReferenceFrame parentFrame, Vector3d comOffset, DoubleYoVariable qx, DoubleYoVariable qy,
              DoubleYoVariable qz, DoubleYoVariable q_qs, DoubleYoVariable q_qx, DoubleYoVariable q_qy, DoubleYoVariable q_qz)
      {
         super(name, parentFrame);
         this.comOffset = comOffset;

         this.qx = qx;
         this.qy = qy;
         this.qz = qz;

         this.q_qs = q_qs;
         this.q_qx = q_qx;
         this.q_qy = q_qy;
         this.q_qz = q_qz;
      }

      @Override
      public void updateTransformToParent(Transform3D transformToParent)
      {
         Vector3d translationBeforeJoint = new Vector3d(qx.getDoubleValue(), qy.getDoubleValue(), qz.getDoubleValue());
         transformToParent.setTranslation(translationBeforeJoint);

         Quat4d quaternion = new Quat4d();
         quaternion.set(q_qx.getDoubleValue(), q_qy.getDoubleValue(), q_qz.getDoubleValue(), q_qs.getDoubleValue());
         Transform3D rotation = new Transform3D();
         rotation.set(quaternion);
         transformToParent.mul(rotation);

         Transform3D translationAfterJoint = new Transform3D();
         translationAfterJoint.setTranslation(comOffset);
         transformToParent.mul(translationAfterJoint);
      }
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
