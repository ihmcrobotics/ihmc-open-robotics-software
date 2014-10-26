package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.junit.Test;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.Line2d;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.test.JUnitTools;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public class CapturePointToolsTest
{
	int nTests = 20;
	Random random = new Random();
	YoVariableRegistry registry = new YoVariableRegistry("");

	@Test
	public void ComputeConstantCMPPointsWithBeginningAndEndBetweenFeetWith2StepsTest()
	{
	   ArrayList<FramePoint> footstepList = new ArrayList<FramePoint>();
      ArrayList<YoFramePoint> arrayToPack = new ArrayList<YoFramePoint>();
      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("test", new FramePose());

      int nFootsteps = 2;
      int numberFootstepsToConsider = 4;
      for (int i = 0; i < numberFootstepsToConsider; i++)
      {
         arrayToPack.add(new YoFramePoint("test" + Integer.toString(i), ReferenceFrame.getWorldFrame(), registry));
      }
      for (int i = 0; i < nFootsteps; i++)
      {
         FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), new Point3d(random.nextDouble(), random.nextDouble(),
               random.nextDouble()),
               new Quat4d(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));

         poseReferenceFrame.setPoseAndUpdate(pose);

         Point3d pointToPack = new Point3d();
         Quat4d orientation = new Quat4d();
         poseReferenceFrame.getPosition(pointToPack);
         poseReferenceFrame.getOrientation(orientation);
         FramePoint footstepData = new FramePoint(poseReferenceFrame.getRootFrame());
         footstepData.set(pointToPack);
         footstepList.add(footstepData);
      }

      CapturePointTools.computeConstantCentersOfPressuresWithBeginningAndEndBetweenFeetRestOnFeet(arrayToPack, footstepList, numberFootstepsToConsider);

      Point3d pointBetweenFeet = new Point3d();
      pointBetweenFeet.set(footstepList.get(0).getPointCopy());
      pointBetweenFeet.add(footstepList.get(1).getPointCopy());
      pointBetweenFeet.scale(0.5);
      
      for (int i = 0; i < arrayToPack.size(); i++)
      {
         JUnitTools.assertPoint3dEquals("", arrayToPack.get(i).getPoint3dCopy(), pointBetweenFeet, 1e-10);
      }
	}
	
	@Test
   public void ComputeConstantCMPPointsOnFeetAndEndBetweenFeetWith2StepsTest()
   {
      ArrayList<FramePoint> footstepList = new ArrayList<FramePoint>();
      ArrayList<YoFramePoint> arrayToPack = new ArrayList<YoFramePoint>();
      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("test", new FramePose());

      int nFootsteps = 2;
      int numberFootstepsToConsider = 4;
      for (int i = 0; i < numberFootstepsToConsider; i++)
      {
         arrayToPack.add(new YoFramePoint("test" + Integer.toString(i), ReferenceFrame.getWorldFrame(), registry));
      }
      for (int i = 0; i < nFootsteps; i++)
      {
         FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), new Point3d(random.nextDouble(), random.nextDouble(),
               random.nextDouble()),
               new Quat4d(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));

         poseReferenceFrame.setPoseAndUpdate(pose);

         Point3d pointToPack = new Point3d();
         Quat4d orientation = new Quat4d();
         poseReferenceFrame.getPosition(pointToPack);
         poseReferenceFrame.getOrientation(orientation);
         FramePoint footstepData = new FramePoint(poseReferenceFrame.getRootFrame());
         footstepData.set(pointToPack);
         footstepList.add(footstepData);
      }

      CapturePointTools.computeConstantCentersOfPressuresOnFeetWithEndBetweenFeet(arrayToPack, footstepList, numberFootstepsToConsider);

      Point3d pointBetweenFeet = new Point3d();
      pointBetweenFeet.set(footstepList.get(0).getPointCopy());
      pointBetweenFeet.add(footstepList.get(1).getPointCopy());
      pointBetweenFeet.scale(0.5);
      
      JUnitTools.assertPoint3dEquals("", footstepList.get(0).getPointCopy(), arrayToPack.get(0).getPoint3dCopy(), 1e-10);
      
      for (int i = 1; i < arrayToPack.size(); i++)
      {
         JUnitTools.assertPoint3dEquals("", arrayToPack.get(i).getPoint3dCopy(), pointBetweenFeet, 1e-10);
      }
   }
	
	@Test
	public void TestComputeConstantCentersOfPressuresOnFeet()
	{
		ArrayList<FramePoint> footstepList = new ArrayList<FramePoint>();
		ArrayList<YoFramePoint> arrayToPack = new ArrayList<YoFramePoint>();
		PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("test", new FramePose());

		int nFootsteps = 10;
		int numberFootstepsToConsider = 3;
		for (int i = 0; i < numberFootstepsToConsider; i++)
		{
			arrayToPack.add(new YoFramePoint("test" + Integer.toString(i), ReferenceFrame.getWorldFrame(), registry));
		}

		for (int j = 0; j < nTests; j++)
		{
			for (int i = 0; i < nFootsteps; i++)
			{
				FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), new Point3d(random.nextDouble(), random.nextDouble(),
						random.nextDouble()),
						new Quat4d(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));

				poseReferenceFrame.setPoseAndUpdate(pose);

				Point3d pointToPack = new Point3d();
				Quat4d orientation = new Quat4d();
				poseReferenceFrame.getPosition(pointToPack);
				poseReferenceFrame.getOrientation(orientation);
				FramePoint footstepData = new FramePoint(poseReferenceFrame.getRootFrame());
				footstepData.set(pointToPack);
				footstepList.add(footstepData);
			}

			CapturePointTools.computeConstantCentersOfPressuresOnFeet(arrayToPack, footstepList, numberFootstepsToConsider);

			for (int i = 0; i < arrayToPack.size(); i++)
			{
				JUnitTools.assertPoint3dEquals("", arrayToPack.get(i).getPoint3dCopy(), footstepList.get(i).getPointCopy(), 1e-10);
			}
		}
	}

	@Test
	public void TestComputeConstantCentersOfPressureWithStartBetweenFeetAndRestOnFeet()
	{
		ArrayList<FramePoint> footstepList = new ArrayList<FramePoint>();
		ArrayList<YoFramePoint> arrayToPack = new ArrayList<YoFramePoint>();
		PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("test", new FramePose());

		int nFootsteps = 10;
		int numberFootstepsToConsider = 3;
		for (int i = 0; i < numberFootstepsToConsider; i++)
		{
			arrayToPack.add(new YoFramePoint("test" + Integer.toString(i), ReferenceFrame.getWorldFrame(), registry));
		}

		for (int j = 0; j < nTests; j++)
		{
			for (int i = 0; i < nFootsteps; i++)
			{
				FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), new Point3d(random.nextDouble(), random.nextDouble(),
						random.nextDouble()),
						new Quat4d(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));

				poseReferenceFrame.setPoseAndUpdate(pose);

				Point3d pointToPack = new Point3d();
				Quat4d orientation = new Quat4d();
				poseReferenceFrame.getPosition(pointToPack);
				poseReferenceFrame.getOrientation(orientation);
				FramePoint footstepData = new FramePoint(poseReferenceFrame.getRootFrame());
				footstepData.set(pointToPack);
				footstepList.add(footstepData);
			}

			CapturePointTools.computeConstantCentersOfPressureWithStartBetweenFeetAndRestOnFeet(arrayToPack, footstepList,
					numberFootstepsToConsider);

			Point3d copPos1 = new Point3d();
			copPos1.set(footstepList.get(0).getPointCopy());
			copPos1.add(footstepList.get(1).getPointCopy());
			copPos1.scale(0.5);
			JUnitTools.assertPoint3dEquals("", copPos1, arrayToPack.get(0).getPoint3dCopy(), 1e-10);
			for (int i = 1; i < arrayToPack.size() - 1; i++)
			{
				JUnitTools.assertPoint3dEquals("", arrayToPack.get(i).getPoint3dCopy(), footstepList.get(i).getPointCopy(), 1e-10);
			}
		}
	}

	@Test
	public void TestComputeConstantCentersOfPressureWithEndBetweenFeetAndRestOnFeet()
	{
		ArrayList<FramePoint> footstepList = new ArrayList<FramePoint>();
		ArrayList<YoFramePoint> arrayToPack = new ArrayList<YoFramePoint>();
		PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("test", new FramePose());

		int nFootsteps = 10;
		int numberFootstepsToConsider = 3;
		for (int i = 0; i < numberFootstepsToConsider; i++)
		{
			arrayToPack.add(new YoFramePoint("test" + Integer.toString(i), ReferenceFrame.getWorldFrame(), registry));
		}

		for (int j = 0; j < nTests; j++)
		{
			for (int i = 0; i < nFootsteps; i++)
			{
				FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), new Point3d(random.nextDouble(), random.nextDouble(),
						random.nextDouble()),
						new Quat4d(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));

				poseReferenceFrame.setPoseAndUpdate(pose);

				Point3d pointToPack = new Point3d();
				Quat4d orientation = new Quat4d();
				poseReferenceFrame.getPosition(pointToPack);
				poseReferenceFrame.getOrientation(orientation);
				FramePoint footstepData = new FramePoint(poseReferenceFrame.getRootFrame());
				footstepData.set(pointToPack);
				footstepList.add(footstepData);
			}

			CapturePointTools.computeConstantCentersOfPressuresOnFeetWithEndBetweenFeet(arrayToPack, footstepList,
					numberFootstepsToConsider);

			Point3d copPos1 = new Point3d();
			copPos1.set(footstepList.get(numberFootstepsToConsider - 1).getPointCopy());
			copPos1.add(footstepList.get(numberFootstepsToConsider - 2).getPointCopy());
			copPos1.scale(0.5);
			JUnitTools.assertPoint3dEquals("", copPos1, arrayToPack.get(numberFootstepsToConsider - 1).getPoint3dCopy(), 1e-10);
			for (int i = 0; i < arrayToPack.size() - 2; i++)
			{
				JUnitTools.assertPoint3dEquals("", arrayToPack.get(i).getPoint3dCopy(), footstepList.get(i).getPointCopy(), 1e-10);
			}
		}
	}

	@Test
	public void TestComputeConstantCentersOfPressureWithEndAndBeginningBetweenFeet()
	{
		ArrayList<FramePoint> footstepList = new ArrayList<FramePoint>();
		ArrayList<YoFramePoint> arrayToPack = new ArrayList<YoFramePoint>();
		PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("test", new FramePose());

		int nFootsteps = 10;
		int numberFootstepsToConsider = 3;
		for (int i = 0; i < numberFootstepsToConsider; i++)
		{
			arrayToPack.add(new YoFramePoint("test" + Integer.toString(i), ReferenceFrame.getWorldFrame(), registry));
		}

		for (int j = 0; j < nTests; j++)
		{
			for (int i = 0; i < nFootsteps; i++)
			{
				FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), new Point3d(random.nextDouble(), random.nextDouble(),
						random.nextDouble()),
						new Quat4d(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));

				poseReferenceFrame.setPoseAndUpdate(pose);

				Point3d pointToPack = new Point3d();
				Quat4d orientation = new Quat4d();
				poseReferenceFrame.getPosition(pointToPack);
				poseReferenceFrame.getOrientation(orientation);
				FramePoint footstepData = new FramePoint(poseReferenceFrame.getRootFrame());

				footstepData.set(pointToPack);
				footstepList.add(footstepData);
			}

			CapturePointTools.computeConstantCentersOfPressuresWithBeginningAndEndBetweenFeetRestOnFeet(arrayToPack, footstepList,
					numberFootstepsToConsider);

			Point3d copPos1 = new Point3d();
			copPos1.set(footstepList.get(numberFootstepsToConsider - 1).getPointCopy());
			copPos1.add(footstepList.get(numberFootstepsToConsider - 2).getPointCopy());
			copPos1.scale(0.5);
			JUnitTools.assertPoint3dEquals("", copPos1, arrayToPack.get(numberFootstepsToConsider - 1).getPoint3dCopy(), 1e-10);

			copPos1.set(footstepList.get(0).getPointCopy());
			copPos1.add(footstepList.get(1).getPointCopy());
			copPos1.scale(0.5);
			JUnitTools.assertPoint3dEquals("", copPos1, arrayToPack.get(0).getPoint3dCopy(), 1e-10);

			for (int i = 1; i < arrayToPack.size() - 2; i++)
			{
				JUnitTools.assertPoint3dEquals("", arrayToPack.get(i).getPoint3dCopy(), footstepList.get(i).getPointCopy(), 1e-10);
			}
		}
	}

	@Test
	public void TestComputeDesiredEndOfStepCapturePointLocations()
	{
		ArrayList<FramePoint> footstepList = new ArrayList<FramePoint>();
		ArrayList<YoFramePoint> constantCentersOfPressures = new ArrayList<YoFramePoint>();
		ArrayList<YoFramePoint> capturePointsToPack = new ArrayList<YoFramePoint>();
		YoFramePoint icpToCheck = new YoFramePoint("icpToCheck", ReferenceFrame.getWorldFrame(), registry);
		PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("test", new FramePose());
		FramePoint2d p1 = new FramePoint2d();
		FramePoint2d p2 = new FramePoint2d();

		int nFootsteps = 10;
		int numberFootstepsToConsider = random.nextInt(((nFootsteps - 3) + 1)) + 3;
		for (int i = 0; i < numberFootstepsToConsider; i++)
		{
			constantCentersOfPressures.add(new YoFramePoint("test" + Integer.toString(i), ReferenceFrame.getWorldFrame(), registry));
		}
		for (int i = 0; i < numberFootstepsToConsider - 1; i++)
		{
			capturePointsToPack.add(new YoFramePoint("testICP" + Integer.toString(i), ReferenceFrame.getWorldFrame(), registry));
		}

		for (int j = 0; j < nTests; j++)
		{
			for (int i = 0; i < numberFootstepsToConsider; i++)
			{
				FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), new Point3d(random.nextDouble(), random.nextDouble(),
						random.nextDouble()),
						new Quat4d(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));

				poseReferenceFrame.setPoseAndUpdate(pose);

				Point3d pointToPack = new Point3d();
				Quat4d orientation = new Quat4d();
				poseReferenceFrame.getPosition(pointToPack);
				poseReferenceFrame.getOrientation(orientation);
				FramePoint footstepData = new FramePoint(poseReferenceFrame.getRootFrame());
				footstepData.set(pointToPack);
				footstepList.add(footstepData);
			}

			CapturePointTools.computeConstantCentersOfPressuresWithBeginningAndEndBetweenFeetRestOnFeet(constantCentersOfPressures,
					footstepList, numberFootstepsToConsider);
			double omega0 = 0.5;
			double time = 0.5;
			CapturePointTools.computeDesiredEndOfStepCapturePointLocations(constantCentersOfPressures, capturePointsToPack, time, omega0);

			CapturePointTools.computeDesiredCapturePointPosition(omega0, 0, capturePointsToPack.get(0), constantCentersOfPressures.get(0),
					icpToCheck);

			JUnitTools.assertFramePointEquals(capturePointsToPack.get(0).getFramePointCopy(), icpToCheck.getFramePointCopy(), 1e-8);

			for (int i = 0; i < constantCentersOfPressures.size() - 2; i++)
			{
				p1.set(constantCentersOfPressures.get(i).getFramePoint2dCopy());
				p2.set(capturePointsToPack.get(i).getFramePoint2dCopy());
				Line2d line = new Line2d(p1.getPointCopy(), p2.getPointCopy());
				p1.set(capturePointsToPack.get(i + 1).getFramePoint2dCopy());
				boolean isPointOnLine = line.isPointOnLine(p1.getPoint());
				assertTrue(isPointOnLine);
			}
		}
	}

	@Test
	public void TestComputeDesiredCapturePointLocations()
	{
		ArrayList<FramePoint> footstepList = new ArrayList<FramePoint>();
		ArrayList<YoFramePoint> constantCentersOfPressures = new ArrayList<YoFramePoint>();
		ArrayList<YoFramePoint> capturePointsToPack = new ArrayList<YoFramePoint>();
		PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("test", new FramePose());
		FramePoint2d p1 = new FramePoint2d();
		FramePoint2d p2 = new FramePoint2d();
		YoFramePoint desiredICP = new YoFramePoint("", ReferenceFrame.getWorldFrame(), registry);

		int nFootsteps = 10;
		int numberFootstepsToConsider = random.nextInt(((nFootsteps - 3) + 1)) + 3;
		for (int i = 0; i < numberFootstepsToConsider; i++)
		{
			constantCentersOfPressures.add(new YoFramePoint("test" + Integer.toString(i), ReferenceFrame.getWorldFrame(), registry));
		}
		for (int i = 0; i < numberFootstepsToConsider - 1; i++)
		{
			capturePointsToPack.add(new YoFramePoint("testICP" + Integer.toString(i), ReferenceFrame.getWorldFrame(), registry));
		}

		for (int j = 0; j < nTests; j++)
		{
			for (int i = 0; i < nFootsteps; i++)
			{
				FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), new Point3d(random.nextDouble(), random.nextDouble(),
						random.nextDouble()),
						new Quat4d(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));

				poseReferenceFrame.setPoseAndUpdate(pose);

				Point3d pointToPack = new Point3d();
				Quat4d orientation = new Quat4d();
				poseReferenceFrame.getPosition(pointToPack);
				poseReferenceFrame.getOrientation(orientation);
				FramePoint footstepData = new FramePoint(poseReferenceFrame.getRootFrame());
				footstepData.set(pointToPack);
				footstepList.add(footstepData);
			}

			CapturePointTools.computeConstantCentersOfPressuresWithBeginningAndEndBetweenFeetRestOnFeet(constantCentersOfPressures,
					footstepList, numberFootstepsToConsider);
			double omega0 = 0.5;
			double time = 0.5;
			CapturePointTools.computeDesiredEndOfStepCapturePointLocations(constantCentersOfPressures, capturePointsToPack, time, omega0);

			for (int i = 0; i < constantCentersOfPressures.size() - 2; i++)
			{
				double timeIntoStep = random.nextDouble() * time;
				CapturePointTools.computeDesiredCapturePointPosition(omega0, timeIntoStep, capturePointsToPack.get(i),
						constantCentersOfPressures.get(i), desiredICP);

				p1.set(constantCentersOfPressures.get(i).getFramePoint2dCopy());
				p2.set(capturePointsToPack.get(i).getFramePoint2dCopy());
				Line2d line = new Line2d(p1.getPointCopy(), p2.getPointCopy());
				p1.set(desiredICP.getFramePoint2dCopy());
				boolean isPointOnLine = line.isPointOnLine(p1.getPoint());
				assertTrue(isPointOnLine);
			}
		}
	}

	@Test
	public void TestComputeDesiredCapturePointVelocity()
	{
		YoFramePoint initialCapturePointPosition = new YoFramePoint("", ReferenceFrame.getWorldFrame(), registry);
		YoFramePoint computedCapturePoint1 = new YoFramePoint("1", ReferenceFrame.getWorldFrame(), registry);
		YoFramePoint computedCapturePoint2 = new YoFramePoint("2", ReferenceFrame.getWorldFrame(), registry);
		YoFramePoint initialCenterOfPressure = new YoFramePoint("3", ReferenceFrame.getWorldFrame(), registry);
		YoFrameVector differentiatedCapturePointPosition = new YoFrameVector("4", ReferenceFrame.getWorldFrame(), registry);
		YoFrameVector computedCapturePointVelocity = new YoFrameVector("5", ReferenceFrame.getWorldFrame(), registry);

		for (int i = 0; i < nTests; i++)
		{
			initialCapturePointPosition.set(random.nextDouble(), random.nextDouble(), 0);
			initialCenterOfPressure.set(initialCapturePointPosition.getX() + 0.02, initialCapturePointPosition.getY() + 0.01, 0);

			double deltaT = 0.001;
			double time = random.nextDouble() * 0.1 + 0.05;
			double omega0 = 0.5;

			CapturePointTools.computeDesiredCapturePointPosition(omega0, time, initialCapturePointPosition, initialCenterOfPressure,
					computedCapturePoint1);
			CapturePointTools.computeDesiredCapturePointPosition(omega0, time + deltaT, initialCapturePointPosition,
					initialCenterOfPressure, computedCapturePoint2);

			differentiatedCapturePointPosition.set(computedCapturePoint2);
			differentiatedCapturePointPosition.sub(computedCapturePoint1);
			differentiatedCapturePointPosition.scale(1 / deltaT);

			CapturePointTools.computeDesiredCapturePointVelocity(omega0, time + deltaT, initialCapturePointPosition,
					initialCenterOfPressure, computedCapturePointVelocity);

			JUnitTools.assertPoint3dEquals("", computedCapturePointVelocity.getPoint3dCopy(),
					differentiatedCapturePointPosition.getPoint3dCopy(), 1e-3);
		}
	}

	@Test
	public void TestComputeDesiredCapturePointAcceleration()
	{
		YoFramePoint initialCapturePointPosition = new YoFramePoint("", ReferenceFrame.getWorldFrame(), registry);
		YoFramePoint initialCenterOfPressure = new YoFramePoint("3", ReferenceFrame.getWorldFrame(), registry);
		YoFrameVector computedCapturePointVelocity = new YoFrameVector("5", ReferenceFrame.getWorldFrame(), registry);
		YoFrameVector desiredCapturePointAcceleration = new YoFrameVector("6", ReferenceFrame.getWorldFrame(), registry);

		for (int i = 0; i < nTests; i++)
		{
			initialCapturePointPosition.set(random.nextDouble(), random.nextDouble(), 0);
			initialCenterOfPressure.set(initialCapturePointPosition.getX() + 0.02, initialCapturePointPosition.getY() + 0.01, 0);

			double time = random.nextDouble() * 0.1 + 0.05;
			double omega0 = 0.5;

			CapturePointTools.computeDesiredCapturePointVelocity(omega0, time, initialCapturePointPosition, initialCenterOfPressure,
					computedCapturePointVelocity);
			CapturePointTools.computeDesiredCapturePointAcceleration(omega0, time, initialCapturePointPosition, initialCenterOfPressure,
					desiredCapturePointAcceleration);
			computedCapturePointVelocity.scale(omega0);

			JUnitTools.assertPoint3dEquals("", computedCapturePointVelocity.getPoint3dCopy(),
					desiredCapturePointAcceleration.getPoint3dCopy(), 1e-10);

			computedCapturePointVelocity.scale(1 / omega0);
			CapturePointTools.computeDesiredCapturePointAcceleration(omega0, computedCapturePointVelocity, desiredCapturePointAcceleration);
			computedCapturePointVelocity.scale(omega0);

			JUnitTools.assertPoint3dEquals("", computedCapturePointVelocity.getPoint3dCopy(),
					desiredCapturePointAcceleration.getPoint3dCopy(), 1e-10);
		}
	}

	@Test
	public void TestComputeInitialCapturePointFromDesiredCapturePointAndInitialCenterOfPressure()
	{
		YoFramePoint constantCenterOfPressure = new YoFramePoint("COP", ReferenceFrame.getWorldFrame(), registry);
		YoFramePoint finalDesiredICP = new YoFramePoint("finalICP", ReferenceFrame.getWorldFrame(), registry);
		YoFramePoint initialICP = new YoFramePoint("initialICP", ReferenceFrame.getWorldFrame(), registry);
		FramePoint2d p1 = new FramePoint2d();
		FramePoint2d p2 = new FramePoint2d();

		for (int j = 0; j < nTests; j++)
		{
			constantCenterOfPressure.set(new Point3d(random.nextDouble(), random.nextDouble(), 0));
			finalDesiredICP.set(new Point3d(random.nextDouble(), random.nextDouble(), 0));

			double omega0 = 3.4;
			double time = 0.5;

			CapturePointTools.computeInitialCapturePointFromDesiredCapturePointAndInitialCenterOfPressure(omega0, time, finalDesiredICP,
					constantCenterOfPressure, initialICP);

			p1.set(constantCenterOfPressure.getFramePoint2dCopy());
			p2.set(finalDesiredICP.getFramePoint2dCopy());
			Line2d line = new Line2d(p1.getPointCopy(), p2.getPointCopy());
			p1.set(initialICP.getFramePoint2dCopy());
			boolean isPointOnLine = line.isPointOnLine(p1.getPoint());
			assertTrue(isPointOnLine);
		}
	}

	@Test
	public void TestComputeConstantCenterOfPressureFromInitialAndFinalCapturePointLocations()
	{
		YoFramePoint constantCenterOfPressure = new YoFramePoint("COP", ReferenceFrame.getWorldFrame(), registry);
		YoFramePoint finalDesiredICP = new YoFramePoint("finalICP", ReferenceFrame.getWorldFrame(), registry);
		YoFramePoint initialICP = new YoFramePoint("initialICP", ReferenceFrame.getWorldFrame(), registry);
		FramePoint2d p1 = new FramePoint2d();
		FramePoint2d p2 = new FramePoint2d();

		for (int j = 0; j < nTests; j++)
		{
			initialICP.set(new Point3d(random.nextDouble(), random.nextDouble(), 0));
			finalDesiredICP.set(new Point3d(random.nextDouble(), random.nextDouble(), 0));

			double omega0 = 3.4;
			double time = 0.5;

			CapturePointTools.computeConstantCenterOfPressureFromInitialAndFinalCapturePointLocations(finalDesiredICP, initialICP,
					constantCenterOfPressure, omega0, time);

			p1.set(initialICP.getFramePoint2dCopy());
			p2.set(finalDesiredICP.getFramePoint2dCopy());
			Line2d line = new Line2d(p1.getPointCopy(), p2.getPointCopy());
			p1.set(constantCenterOfPressure.getFramePoint2dCopy());
			boolean isPointOnLine = line.isPointOnLine(p1.getPoint());
			assertTrue(isPointOnLine);
		}
	}
}
