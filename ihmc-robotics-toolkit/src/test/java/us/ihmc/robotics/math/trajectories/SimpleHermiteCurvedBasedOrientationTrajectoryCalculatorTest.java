package us.ihmc.robotics.math.trajectories;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SimpleHermiteCurvedBasedOrientationTrajectoryCalculatorTest
{
   @Test
   public void compareWithHermiteCurvedBasedOrientationTrajectoryGenerator()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      Random random = new Random(0612L);
      double dt = 0.01;
      double startIntegrationTime = 0.0;
      double endIntegrationTime = 1.0;

      FrameQuaternion currentOrientation = new FrameQuaternion();
      FrameVector3D currentAngularVelocity = new FrameVector3D();
      FrameVector3D currentAngularAcceleration = new FrameVector3D();

      FrameQuaternion initialOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      FrameQuaternion finalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      FrameVector3D initialAngularVelocity = new FrameVector3D(worldFrame);
      FrameVector3D finalAngularVelocity = new FrameVector3D(worldFrame);

      System.out.println("## " + initialOrientation + " " + finalOrientation);

      HermiteCurveBasedOrientationTrajectoryGenerator traj = new HermiteCurveBasedOrientationTrajectoryGenerator("traj", worldFrame,
                                                                                                                 new YoVariableRegistry("null"));
      traj.setTrajectoryTime(endIntegrationTime - startIntegrationTime);
      traj.setInitialConditions(initialOrientation, initialAngularVelocity);
      traj.setFinalConditions(finalOrientation, finalAngularVelocity);
      traj.initialize();

      SimpleHermiteCurvedBasedOrientationTrajectoryCalculator trajCalculator = new SimpleHermiteCurvedBasedOrientationTrajectoryCalculator();
      trajCalculator.setTrajectoryTime(endIntegrationTime - startIntegrationTime);
      trajCalculator.setInitialConditions(initialOrientation, initialAngularVelocity);
      trajCalculator.setFinalConditions(finalOrientation, finalAngularVelocity);
      trajCalculator.setConvertingAngularVelocity(true);
      trajCalculator.setConvertingAngularAcceleration(true);
      trajCalculator.setNumberOfRevolutions(0);
      trajCalculator.initialize();

      for (double time = startIntegrationTime; time <= endIntegrationTime; time += dt)
      {
         traj.compute(time);
         traj.getAngularData(currentOrientation, currentAngularVelocity, currentAngularAcceleration);

         trajCalculator.compute(time);
         Quaternion orientation = new Quaternion();
         Vector3D angularVelocity = new Vector3D();
         Vector3D angularAcceleration = new Vector3D();
         trajCalculator.getOrientation(orientation);
         trajCalculator.getAngularVelocity(angularVelocity);
         trajCalculator.getAngularAcceleration(angularAcceleration);

         boolean angularVelocityEquals = currentAngularVelocity.epsilonEquals(angularVelocity, 0.001);
         boolean angularAccelerationEquals = currentAngularAcceleration.epsilonEquals(angularAcceleration, 0.001);
         assertTrue(angularVelocityEquals, "result is to far from the result of the original trajectory generator.");
         assertTrue(angularAccelerationEquals, "result is to far from the result of the original trajectory generator.");
      }
   }
}
