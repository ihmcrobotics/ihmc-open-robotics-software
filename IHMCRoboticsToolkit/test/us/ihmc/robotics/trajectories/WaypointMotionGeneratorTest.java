package us.ihmc.robotics.trajectories;

import static junit.framework.Assert.assertEquals;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * Created by IntelliJ IDEA.
 * User: Administrator
 * Date: May 14, 2010
 * Time: 3:43:29 PM
 * To change this template use File | Settings | File Templates.
 */
public class WaypointMotionGeneratorTest
{
   public WaypointMotionGeneratorTest()
   {
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testStraightLineMotion()
   {
      ArrayList<FramePoint> listOfPoints = new ArrayList<FramePoint>();

      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();


      FramePoint startPoint = new FramePoint(referenceFrame, 0.0, 0.0, 0.0);
      listOfPoints.add(startPoint);

      FramePoint endPoint = new FramePoint(referenceFrame, 1.0, 0.0, 0.0);
      listOfPoints.add(endPoint);

      ListOfPointsTrajectory listOfPointsTrajectory = new ListOfPointsTrajectory(listOfPoints);

      double moveDuration = 2.0;

      WaypointMotionGenerator waypointMotionGenerator = new WaypointMotionGenerator(listOfPointsTrajectory, moveDuration);

      FramePoint testPoint;

      testPoint = waypointMotionGenerator.getCurrentDesiredPoint(0.0);
      assertEquals(0.0, testPoint.distance(startPoint), 1e-6);

      testPoint = waypointMotionGenerator.getCurrentDesiredPoint(-1.0);
      assertEquals(0.0, testPoint.distance(startPoint), 1e-6);

      testPoint = waypointMotionGenerator.getCurrentDesiredPoint(moveDuration);
      assertEquals(0.0, testPoint.distance(endPoint), 1e-6);

      testPoint = waypointMotionGenerator.getCurrentDesiredPoint(moveDuration + 1.0);
      assertEquals(0.0, testPoint.distance(endPoint), 1e-6);

      FramePoint midPoint = new FramePoint(startPoint);
      midPoint.add(endPoint);
      midPoint.scale(0.5);

      testPoint = waypointMotionGenerator.getCurrentDesiredPoint(moveDuration / 2.0);
      assertEquals(0.0, testPoint.distance(midPoint), 1e-6);

      FrameVector velocity = waypointMotionGenerator.getCurrentDesiredVelocity(0.0);
      assertEquals(0.0, velocity.length(), 1e-6);

      velocity = waypointMotionGenerator.getCurrentDesiredVelocity(moveDuration);
      assertEquals(0.0, velocity.length(), 1e-6);

      FrameVector acceleration = waypointMotionGenerator.getCurrentDesiredAcceleration(0.0);
//      System.err.println("acceleration=" + acceleration);

      assertEquals(0.0, acceleration.length(), 1e-2);

      acceleration = waypointMotionGenerator.getCurrentDesiredAcceleration(moveDuration);

      assertEquals(0.0, acceleration.length(), 1e-2);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testThreePointMotion()
   {
      ArrayList<FramePoint> listOfPoints = new ArrayList<FramePoint>();

      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();


      FramePoint startPoint = new FramePoint(referenceFrame, 0.0, 0.0, 0.0);
      listOfPoints.add(startPoint);

      FramePoint middlePoint = new FramePoint(referenceFrame, 1.0, 1.0, 0.0);
      listOfPoints.add(middlePoint);


      FramePoint endPoint = new FramePoint(referenceFrame, 2.0, 0.0, 0.0);
      listOfPoints.add(endPoint);

      ListOfPointsTrajectory listOfPointsTrajectory = new ListOfPointsTrajectory(listOfPoints);

      double moveDuration = 2.0;

      WaypointMotionGenerator waypointMotionGenerator = new WaypointMotionGenerator(listOfPointsTrajectory, moveDuration);

      FramePoint testPoint;

      testPoint = waypointMotionGenerator.getCurrentDesiredPoint(moveDuration / 2.0);
      assertEquals(0.0, testPoint.distance(middlePoint), 1e-6);
   }




}
