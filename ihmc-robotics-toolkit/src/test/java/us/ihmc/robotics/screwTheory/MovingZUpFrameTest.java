package us.ihmc.robotics.screwTheory;

import static org.junit.Assert.*;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Random;
import java.util.Set;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.referenceFrames.ZUpFrame;

public class MovingZUpFrameTest
{
   private static final double EPSILON = 1.0e-7;

   @Test
   public void testYawShortcut() throws Exception
   {
      Random random = new Random(5646);

      for (int i = 0; i < 5000; i++)
      { // This test was used when implementing the class, not sure if still useful
         RotationMatrix rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);

         RotationMatrix expectedZUp = new RotationMatrix();
         expectedZUp.setToYawMatrix(rotationMatrix.getYaw());

         RotationMatrix actualZUp = new RotationMatrix();
         double sinPitch = -rotationMatrix.getM20();
         double cosPitch = Math.sqrt(1.0 - sinPitch * sinPitch);
         double cosYaw = rotationMatrix.getM00() / cosPitch;
         double sinYaw = rotationMatrix.getM10() / cosPitch;
         actualZUp.set(cosYaw, -sinYaw, 0.0, sinYaw, cosYaw, 0.0, 0.0, 0.0, 1.0);

         EuclidCoreTestTools.assertMatrix3DEquals(expectedZUp, actualZUp, EPSILON);
      }
      
      for (int i = 0; i < 5000; i++)
      { // This test the reference frame implementation against what it is suppose to do: cancel out the pitch and roll of the original frame.
         RigidBodyTransform originalTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         MovingReferenceFrame randomMovingFrame = new MovingReferenceFrame("blop", ReferenceFrame.getWorldFrame())
         {
            @Override
            protected void updateTransformToParent(RigidBodyTransform transformToParent)
            {
               transformToParent.set(originalTransform);
            }
            
            @Override
            protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
            {
            }
         };

         MovingZUpFrame zUpFrame = new MovingZUpFrame(randomMovingFrame, "blopButZUp");

         randomMovingFrame.update();
         zUpFrame.update();

         RigidBodyTransform expectedTransform = new RigidBodyTransform();
         expectedTransform.setTranslation(originalTransform.getTranslationVector());
         expectedTransform.setRotationYaw(originalTransform.getRotationMatrix().getYaw());

         EuclidCoreTestTools.assertRigidBodyTransformEquals(expectedTransform, zUpFrame.getTransformToWorldFrame(), EPSILON);
      }
   }

   @Test
   public void testYawRateShortcut()
   {
      Random random = new Random(234523);

      for (int i = 0; i < 5000; i++)
      { // This test was used when implementing the class, not sure if still useful
         RotationMatrix rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         Vector3D angularVelocity = EuclidCoreRandomTools.nextVector3D(random);

         double[] yawPitchRoll = new double[3];
         rotationMatrix.getYawPitchRoll(yawPitchRoll);

         double sinPitch = -rotationMatrix.getM20();
         double cosPitch = Math.sqrt(1.0 - sinPitch * sinPitch);
         double cosYaw = rotationMatrix.getM00() / cosPitch;
         double sinYaw = rotationMatrix.getM10() / cosPitch;
         double cosRoll = rotationMatrix.getM22() / cosPitch;
         double sinRoll = rotationMatrix.getM21() / cosPitch;

         assertEquals(sinPitch, Math.sin(yawPitchRoll[1]), EPSILON);
         assertEquals(cosPitch, Math.cos(yawPitchRoll[1]), EPSILON);
         assertEquals(sinYaw, Math.sin(yawPitchRoll[0]), EPSILON);
         assertEquals(cosYaw, Math.cos(yawPitchRoll[0]), EPSILON);
         assertEquals(sinRoll, Math.sin(yawPitchRoll[2]), EPSILON);
         assertEquals(cosRoll, Math.cos(yawPitchRoll[2]), EPSILON);

         double actuaYawRate = (sinRoll * angularVelocity.getY() + cosRoll * angularVelocity.getZ()) / cosPitch;
         double expectedYawRate = RotationTools.computeYawRate(yawPitchRoll, angularVelocity, true);

         assertEquals(expectedYawRate, actuaYawRate, EPSILON);
      }
      
      for (int i = 0; i < 5000; i++)
      { // This test the reference frame implementation against what it is suppose to do: cancel out the pitch rate and roll rate of the original frame.
         RigidBodyTransform originalTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Vector3D originalAngularVelocity = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D originalLinearVelocity = EuclidCoreRandomTools.nextVector3D(random);

         MovingReferenceFrame randomMovingFrame = new MovingReferenceFrame("blop", ReferenceFrame.getWorldFrame())
         {
            @Override
            protected void updateTransformToParent(RigidBodyTransform transformToParent)
            {
               transformToParent.set(originalTransform);
            }
            
            @Override
            protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
            {
               twistRelativeToParentToPack.setToZero(this, getParent(), this);
               twistRelativeToParentToPack.setAngularPart(originalAngularVelocity);
               twistRelativeToParentToPack.setLinearPart(originalLinearVelocity);
            }
         };

         MovingZUpFrame zUpFrame = new MovingZUpFrame(randomMovingFrame, "blopButZUp");

         randomMovingFrame.update();
         zUpFrame.update();

         double[] yawPitchRoll = new double[3];
         originalTransform.getRotationYawPitchRoll(yawPitchRoll);
         Twist expectedTwist = new Twist(zUpFrame, ReferenceFrame.getWorldFrame(), randomMovingFrame);
         expectedTwist.setLinearPart(originalLinearVelocity);
         expectedTwist.changeFrame(zUpFrame);
         expectedTwist.setAngularPartZ(RotationTools.computeYawRate(yawPitchRoll, originalAngularVelocity, true));

         assertTrue(expectedTwist.epsilonEquals(zUpFrame.getTwistOfFrame(), EPSILON));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAgainstFiniteDifferenceWithChainRobot()
   {
      Random random = new Random(3452345L);
      int numberOfJoints = 20;
      double updateDT = 1.0e-8;

      List<OneDoFJoint> joints = ScrewTestTools.createRandomChainRobotWithOneDoFJoints(numberOfJoints, random);

      Map<MovingZUpFrame, NumericalMovingReferenceFrame> jointFramesToFDFrames = new HashMap<>();
      for (OneDoFJoint joint : joints)
      {
         MovingReferenceFrame frameAfterJoint = joint.getFrameAfterJoint();

         MovingZUpFrame zupFrameAfterJoint = new MovingZUpFrame(frameAfterJoint, frameAfterJoint.getName() + "ZUp");

         NumericalMovingReferenceFrame frameAfterJointFD = new NumericalMovingReferenceFrame(zupFrameAfterJoint, updateDT);
         jointFramesToFDFrames.put(zupFrameAfterJoint, frameAfterJointFD);
      }

      Twist actualTwist = new Twist();
      Twist expectedTwist = new Twist();

      ScrewTestTools.setRandomPositions(joints, random);
      ScrewTestTools.setRandomVelocities(joints, random);
      joints.get(0).getPredecessor().updateFramesRecursively();

      jointFramesToFDFrames.keySet().forEach(MovingReferenceFrame::update);
      jointFramesToFDFrames.values().forEach(MovingReferenceFrame::update);

      for (int i = 0; i < 100; i++)
      {
         ScrewTestTools.integrateVelocities(joints, updateDT);
         joints.get(0).getPredecessor().updateFramesRecursively();
         jointFramesToFDFrames.keySet().forEach(MovingReferenceFrame::update);
         jointFramesToFDFrames.values().forEach(MovingReferenceFrame::update);

         Set<Entry<MovingZUpFrame, NumericalMovingReferenceFrame>> entrySet = jointFramesToFDFrames.entrySet();
         for (Entry<MovingZUpFrame, NumericalMovingReferenceFrame> entry : entrySet)
         {
            entry.getKey().getTwistOfFrame(expectedTwist);
            entry.getValue().getTwistOfFrame(actualTwist);
            expectedTwist.changeBodyFrameNoRelativeTwist(entry.getValue());
            expectedTwist.changeFrame(entry.getValue());

            TwistCalculatorTest.assertTwistEquals(expectedTwist, actualTwist, 1.0e-5);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConsistencyWithZUpFrameWithChainRobot()
   {
      Random random = new Random(3452345L);
      int numberOfJoints = 20;

      List<OneDoFJoint> joints = ScrewTestTools.createRandomChainRobotWithOneDoFJoints(numberOfJoints, random);

      Map<ZUpFrame, MovingZUpFrame> zUpFramesToMovingZUpFrames = new HashMap<>();
      for (OneDoFJoint joint : joints)
      {
         MovingReferenceFrame frameAfterJoint = joint.getFrameAfterJoint();

         ZUpFrame zUpFrameAfterJoint = new ZUpFrame(frameAfterJoint.getRootFrame(), frameAfterJoint, frameAfterJoint.getName() + "ZUp");
         MovingZUpFrame movingZUpFrameAfterJoint = new MovingZUpFrame(frameAfterJoint, frameAfterJoint.getName() + "ZUp");

         zUpFramesToMovingZUpFrames.put(zUpFrameAfterJoint, movingZUpFrameAfterJoint);
      }

      RigidBodyTransform actualTransform = new RigidBodyTransform();
      RigidBodyTransform expectedTransform = new RigidBodyTransform();

      for (int i = 0; i < 100; i++)
      {
         ScrewTestTools.setRandomPositions(joints, random);
         joints.get(0).getPredecessor().updateFramesRecursively();
         zUpFramesToMovingZUpFrames.keySet().forEach(ReferenceFrame::update);
         zUpFramesToMovingZUpFrames.values().forEach(ReferenceFrame::update);

         Set<Entry<ZUpFrame, MovingZUpFrame>> entrySet = zUpFramesToMovingZUpFrames.entrySet();
         for (Entry<ZUpFrame, MovingZUpFrame> entry : entrySet)
         {
            expectedTransform.set(entry.getKey().getTransformToRoot());
            actualTransform.set(entry.getValue().getTransformToRoot());

            EuclidCoreTestTools.assertRigidBodyTransformEquals(expectedTransform, actualTransform, 1.0e-12);
         }
      }
   }
}
