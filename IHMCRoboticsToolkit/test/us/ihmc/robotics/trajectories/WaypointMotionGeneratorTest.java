package us.ihmc.robotics.trajectories;

import static junit.framework.Assert.assertEquals;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

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
      ArrayList<FramePoint3D> listOfPoints = new ArrayList<FramePoint3D>();

      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();


      FramePoint3D startPoint = new FramePoint3D(referenceFrame, 0.0, 0.0, 0.0);
      listOfPoints.add(startPoint);

      FramePoint3D endPoint = new FramePoint3D(referenceFrame, 1.0, 0.0, 0.0);
      listOfPoints.add(endPoint);

      ListOfPointsTrajectory listOfPointsTrajectory = new ListOfPointsTrajectory(listOfPoints);

      double moveDuration = 2.0;

      WaypointMotionGenerator waypointMotionGenerator = new WaypointMotionGenerator(listOfPointsTrajectory, moveDuration);

      FramePoint3D testPoint;

      testPoint = waypointMotionGenerator.getCurrentDesiredPoint(0.0);
      assertEquals(0.0, testPoint.distance(startPoint), 1e-6);

      testPoint = waypointMotionGenerator.getCurrentDesiredPoint(-1.0);
      assertEquals(0.0, testPoint.distance(startPoint), 1e-6);

      testPoint = waypointMotionGenerator.getCurrentDesiredPoint(moveDuration);
      assertEquals(0.0, testPoint.distance(endPoint), 1e-6);

      testPoint = waypointMotionGenerator.getCurrentDesiredPoint(moveDuration + 1.0);
      assertEquals(0.0, testPoint.distance(endPoint), 1e-6);

      FramePoint3D midPoint = new FramePoint3D(startPoint);
      midPoint.add(endPoint);
      midPoint.scale(0.5);

      testPoint = waypointMotionGenerator.getCurrentDesiredPoint(moveDuration / 2.0);
      assertEquals(0.0, testPoint.distance(midPoint), 1e-6);

      FrameVector3D velocity = waypointMotionGenerator.getCurrentDesiredVelocity(0.0);
      assertEquals(0.0, velocity.length(), 1e-6);

      velocity = waypointMotionGenerator.getCurrentDesiredVelocity(moveDuration);
      assertEquals(0.0, velocity.length(), 1e-6);

      FrameVector3D acceleration = waypointMotionGenerator.getCurrentDesiredAcceleration(0.0);
//      System.err.println("acceleration=" + acceleration);

      assertEquals(0.0, acceleration.length(), 1e-2);

      acceleration = waypointMotionGenerator.getCurrentDesiredAcceleration(moveDuration);

      assertEquals(0.0, acceleration.length(), 1e-2);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testThreePointMotion()
   {
      ArrayList<FramePoint3D> listOfPoints = new ArrayList<FramePoint3D>();

      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();


      FramePoint3D startPoint = new FramePoint3D(referenceFrame, 0.0, 0.0, 0.0);
      listOfPoints.add(startPoint);

      FramePoint3D middlePoint = new FramePoint3D(referenceFrame, 1.0, 1.0, 0.0);
      listOfPoints.add(middlePoint);


      FramePoint3D endPoint = new FramePoint3D(referenceFrame, 2.0, 0.0, 0.0);
      listOfPoints.add(endPoint);

      ListOfPointsTrajectory listOfPointsTrajectory = new ListOfPointsTrajectory(listOfPoints);

      double moveDuration = 2.0;

      WaypointMotionGenerator waypointMotionGenerator = new WaypointMotionGenerator(listOfPointsTrajectory, moveDuration);

      FramePoint3D testPoint;

      testPoint = waypointMotionGenerator.getCurrentDesiredPoint(moveDuration / 2.0);
      assertEquals(0.0, testPoint.distance(middlePoint), 1e-6);
   }




}
