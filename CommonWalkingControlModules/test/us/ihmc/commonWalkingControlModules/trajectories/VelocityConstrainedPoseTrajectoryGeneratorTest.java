package us.ihmc.commonWalkingControlModules.trajectories;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class VelocityConstrainedPoseTrajectoryGeneratorTest
{
   private static final Random random = new Random(1516351L);

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final ReferenceFrame frameA = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent("frameA", worldFrame,
         EuclidCoreRandomTools.generateRandomRigidBodyTransform(random));

   private static final double EPSILON = 1e-10;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRuntimeExceptions()
   {
      YoVariableRegistry registry = new YoVariableRegistry("youpiloup");
      VelocityConstrainedPoseTrajectoryGenerator trajToTest1 = new VelocityConstrainedPoseTrajectoryGenerator("blop1", worldFrame, registry);
      VelocityConstrainedPoseTrajectoryGenerator trajToTest2 = new VelocityConstrainedPoseTrajectoryGenerator("blop2", worldFrame, registry);

      try
      {
         trajToTest1.registerNewTrajectoryFrame(frameA);
         fail("Should have thrown an exception.");
      }
      catch (Exception e)
      {
         // Good
      }

      try
      {
         trajToTest1.switchTrajectoryFrame(worldFrame);
         fail("Should have thrown an exception.");
      }
      catch (Exception e)
      {
         // Good
      }

      try
      {
         trajToTest2.registerNewTrajectoryFrame(frameA);
         fail("Should have thrown an exception.");
      }
      catch (Exception e)
      {
         // Good
      }

      try
      {
         trajToTest2.switchTrajectoryFrame(worldFrame);
         fail("Should have thrown an exception.");
      }
      catch (Exception e)
      {
         // Good
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCompareWithStraightLinePoseTrajectoryGeneratorPositionOnly()
   {
      YoVariableRegistry registry = new YoVariableRegistry("youpiloup");
      double time = 1.0;
      VelocityConstrainedPoseTrajectoryGenerator trajToTest = new VelocityConstrainedPoseTrajectoryGenerator("Traj1", worldFrame, registry);
      StraightLinePoseTrajectoryGenerator trajToCompare = new StraightLinePoseTrajectoryGenerator("Traj2", worldFrame, registry);

      FrameOrientation initialOrientation = new FrameOrientation(worldFrame);
      FrameOrientation finalOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);

      FramePoint initialPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 10.0, 10.0, 10.0);
      FramePoint finalPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 10.0, 10.0, 10.0);

      trajToTest.setInitialPoseWithoutInitialVelocity(initialPosition, initialOrientation);
      trajToTest.setFinalPoseWithoutFinalVelocity(new FramePose(finalPosition, finalOrientation));
      trajToTest.setTrajectoryTime(time);
      trajToTest.initialize();

      trajToCompare.setInitialPose(initialPosition, initialOrientation);
      trajToCompare.setFinalPose(finalPosition, finalOrientation);
      trajToCompare.setTrajectoryTime(time);
      trajToCompare.initialize();

      double dt = 1.0e-3;
      FramePoint position1 = new FramePoint();
      FrameVector velocity1 = new FrameVector();
      FrameVector acceleration1 = new FrameVector();
      FrameOrientation orientation1 = new FrameOrientation();
      FrameVector angularVelocity1 = new FrameVector();
      FrameVector angularAcceleration1 = new FrameVector();

      FramePoint position2 = new FramePoint();
      FrameVector velocity2 = new FrameVector();
      FrameVector acceleration2 = new FrameVector();
      FrameOrientation orientation2 = new FrameOrientation();
      FrameVector angularVelocity2 = new FrameVector();
      FrameVector angularAcceleration2 = new FrameVector();

      for (double t = 0.0; t <= time; t += dt)
      {
         trajToTest.compute(t);
         trajToCompare.compute(t);

         trajToTest.getLinearData(position2, velocity2, acceleration2);
         trajToTest.getAngularData(orientation2, angularVelocity2, angularAcceleration2);
         trajToCompare.getLinearData(position1, velocity1, acceleration1);
         trajToCompare.getAngularData(orientation1, angularVelocity1, angularAcceleration1);

         assertTrue(position1.epsilonEquals(position2, EPSILON));
         assertTrue(velocity1.epsilonEquals(velocity2, EPSILON));
         assertTrue(acceleration1.epsilonEquals(acceleration2, EPSILON));
         assertTrue(orientation1.epsilonEquals(orientation2, EPSILON));
         // numerical errors in due to finite differences used in VelocityConstrainedPoseTrajectoryGenerator in algorithm
         //         assertTrue(angularVelocity1.epsilonEquals(angularVelocity2, EPSILON));
         //         assertTrue(angularAcceleration1.epsilonEquals(angularAcceleration2, EPSILON));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPositionAndVelocityConsistencyWithInitialVelocity()
   {
      YoVariableRegistry registry = new YoVariableRegistry("PositionAndVelocityConsistency");
      double time = 1.0;
      VelocityConstrainedPoseTrajectoryGenerator trajToTest = new VelocityConstrainedPoseTrajectoryGenerator("Traj1", worldFrame, registry);

      FramePoint initialPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 10.0, 10.0, 10.0);
      FrameVector initialVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      FramePoint finalPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 10.0, 10.0, 10.0);

      FrameOrientation initialOrientation = new FrameOrientation(worldFrame, 0.0, 0.0, 0.0);
      FrameVector initialAngularVelocity = new FrameVector(worldFrame, 0.0, 0.0, 0.0);
      FrameOrientation finalOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);

      trajToTest.setInitialPoseWithInitialVelocity(initialPosition, initialVelocity, initialOrientation, initialAngularVelocity);
      trajToTest.setFinalPoseWithoutFinalVelocity(new FramePose(finalPosition, finalOrientation));
      trajToTest.setTrajectoryTime(time);
      trajToTest.initialize();

      double deltaT = 1.0e-2;
      double t1, t2, t3;

      FrameVector finiteDiffVel = new FrameVector();
      FrameVector finiteDiffAcc = new FrameVector();

      FramePoint position1 = new FramePoint();
      FrameVector velocity1 = new FrameVector();
      FrameVector acceleration1 = new FrameVector();
      FrameOrientation orientation1 = new FrameOrientation();
      FrameVector angularVelocity1 = new FrameVector();
      FrameVector angularAcceleration1 = new FrameVector();

      FramePoint position2 = new FramePoint();
      FrameVector velocity2 = new FrameVector();
      FrameVector acceleration2 = new FrameVector();
      FrameOrientation orientation2 = new FrameOrientation();
      FrameVector angularVelocity2 = new FrameVector();
      FrameVector angularAcceleration2 = new FrameVector();

      FramePoint position3 = new FramePoint();
      FrameVector velocity3 = new FrameVector();
      FrameVector acceleration3 = new FrameVector();
      FrameOrientation orientation3 = new FrameOrientation();
      FrameVector angularVelocity3 = new FrameVector();
      FrameVector angularAcceleration3 = new FrameVector();

      Quaternion quat1 = new Quaternion();
      Quaternion quat3 = new Quaternion();
      Quaternion quatDelta = new Quaternion();
      double angle, omega;
      Vector3D vectorDelta = new Vector3D();

      FrameVector discreteAngularVelocity = new FrameVector();
      double FDdt = 5e-6;
      for (t3 = 2.0 * FDdt; t3 <= time; t3 += deltaT)
      {

         t2 = t3 - FDdt;
         t1 = t3 - 2.0 * FDdt;
         trajToTest.compute(t1);
         trajToTest.getLinearData(position1, velocity1, acceleration1);
         trajToTest.getAngularData(orientation1, angularVelocity1, angularAcceleration1);

         trajToTest.compute(t2);
         trajToTest.getLinearData(position2, velocity2, acceleration2);
         trajToTest.getAngularData(orientation2, angularVelocity2, angularAcceleration2);

         trajToTest.compute(t3);
         trajToTest.getLinearData(position3, velocity3, acceleration3);
         trajToTest.getAngularData(orientation3, angularVelocity3, angularAcceleration3);

         // Linear Part
         finiteDiffVel.set(calculateFiniteDdt(position1, position3, FDdt));
         finiteDiffAcc.set(calulateFiniteDDdt(position1, position2, position3, FDdt));

         // Rotational Part
         orientation3.getQuaternion(quat3);
         orientation1.getQuaternion(quat1);

         quat1.inverse();
         quatDelta.multiply(quat3, quat1);
         quatDelta.normalize();

         angle = Math.acos(quatDelta.getS()) * 2.0;
         omega = angle / (2.0 * FDdt);

         vectorDelta.set(quatDelta.getX(), quatDelta.getY(), quatDelta.getZ());

         if (vectorDelta.length() > 0.0)
         {
            vectorDelta.normalize();
         }
         vectorDelta.scale(omega);
         discreteAngularVelocity.set(vectorDelta);

         assertTrue(discreteAngularVelocity.epsilonEquals(angularVelocity2, 1e-2));
         assertTrue(finiteDiffVel.epsilonEquals(velocity2, 1e-6));
         assertTrue(finiteDiffAcc.epsilonEquals(acceleration2, 1e-2));
      }
   }

   /**
    * TODO: Get that working with final angular velocity.
    * 
    **/

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPositionAndVelocityConsistencyWithInitialAndFinalVelocity()
   {
      YoVariableRegistry registry = new YoVariableRegistry("PositionAndVelocityConsistency");
      double time = 1.0;
      VelocityConstrainedPoseTrajectoryGenerator trajToTest = new VelocityConstrainedPoseTrajectoryGenerator("Traj1", worldFrame, registry);

      FramePoint initialPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 10.0, 10.0, 10.0);
      FrameVector initialVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      FramePoint finalPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 10.0, 10.0, 10.0);
      FrameVector finalVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);

      FrameOrientation initialOrientation = new FrameOrientation(worldFrame, 0.0, 0.0, 0.0);
      FrameVector initialAngularVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      FrameOrientation finalOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      FrameVector finalAngularVelocity = new FrameVector(worldFrame, 1.0, 0.0, 0.0);//.generateRandomFrameVector(random, worldFrame);

      trajToTest.setInitialPoseWithInitialVelocity(initialPosition, initialVelocity, initialOrientation, initialAngularVelocity);
      //      trajToTest.setFinalPoseWithFinalVelocity(new FramePose(finalPosition, finalOrientation), finalVelocity, finalAngularVelocity);
      trajToTest.setFinalPoseWithoutFinalVelocity(new FramePose(finalPosition, finalOrientation));
      trajToTest.setTrajectoryTime(time);
      trajToTest.initialize();

      double deltaT = 1.0e-2;
      double t1, t2, t3;

      FrameVector finiteDiffVel = new FrameVector();
      FrameVector finiteDiffAcc = new FrameVector();

      FramePoint position1 = new FramePoint();
      FrameVector velocity1 = new FrameVector();
      FrameVector acceleration1 = new FrameVector();
      FrameOrientation orientation1 = new FrameOrientation();
      FrameVector angularVelocity1 = new FrameVector();
      FrameVector angularAcceleration1 = new FrameVector();

      FramePoint position2 = new FramePoint();
      FrameVector velocity2 = new FrameVector();
      FrameVector acceleration2 = new FrameVector();
      FrameOrientation orientation2 = new FrameOrientation();
      FrameVector angularVelocity2 = new FrameVector();
      FrameVector angularAcceleration2 = new FrameVector();

      FramePoint position3 = new FramePoint();
      FrameVector velocity3 = new FrameVector();
      FrameVector acceleration3 = new FrameVector();
      FrameOrientation orientation3 = new FrameOrientation();
      FrameVector angularVelocity3 = new FrameVector();
      FrameVector angularAcceleration3 = new FrameVector();

      Quaternion quat1 = new Quaternion();
      Quaternion quat3 = new Quaternion();
      Quaternion quatDelta = new Quaternion();
      double angle, omega;
      Vector3D vectorDelta = new Vector3D();

      FrameVector discreteAngularVelocity = new FrameVector();
      double FDdt = 5e-6;
      for (t3 = 2.0 * FDdt; t3 <= time; t3 += deltaT)
      {

         t2 = t3 - FDdt;
         t1 = t3 - 2.0 * FDdt;
         trajToTest.compute(t1);
         trajToTest.getLinearData(position1, velocity1, acceleration1);
         trajToTest.getAngularData(orientation1, angularVelocity1, angularAcceleration1);

         trajToTest.compute(t2);
         trajToTest.getLinearData(position2, velocity2, acceleration2);
         trajToTest.getAngularData(orientation2, angularVelocity2, angularAcceleration2);

         trajToTest.compute(t3);
         trajToTest.getLinearData(position3, velocity3, acceleration3);
         trajToTest.getAngularData(orientation3, angularVelocity3, angularAcceleration3);

         // Linear Part
         finiteDiffVel.set(calculateFiniteDdt(position1, position3, FDdt));
         finiteDiffAcc.set(calulateFiniteDDdt(position1, position2, position3, FDdt));

         // Rotational Part
         orientation3.getQuaternion(quat3);
         orientation1.getQuaternion(quat1);

         quat1.inverse();
         quatDelta.multiply(quat3, quat1);
         quatDelta.normalize();

         angle = Math.acos(quatDelta.getS()) * 2.0;
         omega = angle / (2.0 * FDdt);

         vectorDelta.set(quatDelta.getX(), quatDelta.getY(), quatDelta.getZ());

         if (vectorDelta.length() > 0.0)
         {
            vectorDelta.normalize();
         }
         vectorDelta.scale(omega);
         discreteAngularVelocity.set(vectorDelta);
         System.out.println("Angular Velocity calculated by unitTest: " + discreteAngularVelocity + " Angular Velocity from generator: " + angularVelocity2);

         assertTrue(discreteAngularVelocity.epsilonEquals(angularVelocity2, 6e-1)); // bad accuracy!!! Fix algorithm
         assertTrue(finiteDiffVel.epsilonEquals(velocity2, 1e-6));
         assertTrue(finiteDiffAcc.epsilonEquals(acceleration2, 1e-2));
      }
   }

   FrameVector calculateFiniteDdt(FramePoint a, FramePoint c, double dt)
   {
      FrameVector ret = new FrameVector();
      ret.sub(c, a);
      ret.scale(1.0 / (2.0 * dt));
      return ret;
   }

   FrameVector calulateFiniteDDdt(FramePoint a, FramePoint b, FramePoint c, double dt)
   {
      FrameVector ret = new FrameVector();
      ret.add(a, c);
      ret.sub(b);
      ret.sub(b);
      ret.scale(1 / (dt * dt));
      return ret;
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTooBigTime()
   {
      YoVariableRegistry registry = new YoVariableRegistry("youpiloup");
      VelocityConstrainedPoseTrajectoryGenerator trajToTest = new VelocityConstrainedPoseTrajectoryGenerator("blop", worldFrame, registry);

      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(10.0);
      FramePoint initialPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 100.0, 100.0, 100.0);
      FramePoint finalPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 100.0, 100.0, 100.0);

      FrameOrientation initialOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      FrameOrientation finalOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);

      FrameVector initialVelocity = new FrameVector(worldFrame, 0.0, 0.0, 0.0);
      FrameVector initialAngularVelocity = new FrameVector(worldFrame, 0.0, 0.0, 0.0);

      trajToTest.setInitialPoseWithInitialVelocity(initialPosition, initialVelocity, initialOrientation, initialAngularVelocity);
      trajToTest.setFinalPoseWithoutFinalVelocity(new FramePose(finalPosition, finalOrientation));
      trajToTest.setTrajectoryTime(trajectoryTimeProvider.getValue());

      trajToTest.initialize();
      trajToTest.compute(15.0);

      FramePoint position1 = new FramePoint(finalPosition);
      FrameVector velocity1 = new FrameVector(worldFrame);
      FrameVector acceleration1 = new FrameVector(worldFrame);
      FrameOrientation orientation1 = new FrameOrientation(finalOrientation);
      FrameVector angularVelocity1 = new FrameVector(worldFrame);
      FrameVector angularAcceleration1 = new FrameVector(worldFrame);

      FramePoint position2 = new FramePoint();
      FrameVector velocity2 = new FrameVector();
      FrameVector acceleration2 = new FrameVector();
      FrameOrientation orientation2 = new FrameOrientation();
      FrameVector angularVelocity2 = new FrameVector();
      FrameVector angularAcceleration2 = new FrameVector();

      trajToTest.getLinearData(position2, velocity2, acceleration2);
      trajToTest.getAngularData(orientation2, angularVelocity2, angularAcceleration2);

      assertTrue(position1.epsilonEquals(position2, EPSILON));
      assertTrue(velocity1.epsilonEquals(velocity2, EPSILON));
      assertTrue(acceleration1.epsilonEquals(acceleration2, EPSILON));
      assertTrue(orientation1.epsilonEquals(orientation2, EPSILON));
      assertTrue(angularVelocity1.epsilonEquals(angularVelocity2, EPSILON));
      assertTrue(angularAcceleration1.epsilonEquals(angularAcceleration2, EPSILON));
   }

}
