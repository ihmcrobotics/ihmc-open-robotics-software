package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import static org.junit.Assert.assertEquals;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import org.junit.After;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.PointAndLinePlotter;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.robotics.lists.FrameTupleArrayList;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.utilities.test.JUnitTools;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFrameLineSegment2d;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;

public class NewInstantaneousCapturePointPlannerWithSmootherTest
{
	private YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

	private boolean visualize = false;
	private final Random random = new Random();

	private PointAndLinePlotter pointAndLinePlotter = new PointAndLinePlotter(registry);
	private YoGraphicsListRegistry yoGraphicsListRegistry = null;
	private SimulationConstructionSet scs = null;
	private DoubleYoVariable timeYoVariable = null;

	YoFramePoint tmpPreviousICPPosition = new YoFramePoint("PreviousICPPosition", ReferenceFrame.getWorldFrame(), registry);

	private YoFramePoint icpArrowTip = null;
	private YoFramePoint icpPositionYoFramePoint = null;
	private YoFramePoint singleSupportInitialICPPosition = null;
	private YoFramePoint singleSupportFinalICPPosition = null;
	private YoFramePoint icpVelocityYoFramePoint = null;
	private YoFramePoint cmpPositionYoFramePoint = null;

	private ArrayList<YoFramePoint> footstepYoFramePoints = null;

	private ArrayList<YoFramePoint> icpFootCenterCornerPointsViz = null;

	private ArrayList<YoFramePoint> constantCoPsViz = null;

	private YoFramePoint doubleSupportStartICPYoFramePoint = null;
	private YoFramePoint doubleSupportEndICPYoFramePoint = null;

	private final double deltaT = 0.001;

	private final CapturePointPlannerParameters testICPPlannerParams = new CapturePointPlannerParameters()
	{

		@Override
		public double getSingleSupportDuration()
		{
			return 0.7;
		}

		@Override
		public int getNumberOfFootstepsToConsider()
		{
			return 3;
		}

		@Override
		public int getNumberOfCoefficientsForDoubleSupportPolynomialTrajectory()
		{
			return 5;
		}

		@Override
		public double getDoubleSupportInitialTransferDuration()
		{
			return 1.0;
		}

		@Override
		public double getDoubleSupportDuration()
		{
			return 0.25;
		}

	   @Override
	   public double getAdditionalTimeForSingleSupport()
	   {
	      return 0.0;
	   }

		@Override
		public int getNumberOfFootstepsToStop()
		{
			return 2;
		}

		@Override
		public double getIsDoneTimeThreshold()
		{
			return -1e-4;
		}

		@Override
		public double getDoubleSupportSplitFraction()
		{
			return 0.0;
		}

		@Override
		public double getFreezeTimeFactor()
		{
			return 0.9;
		}

		@Override
		public double getMaxInstantaneousCapturePointErrorForStartingSwing()
		{
			return 0.02;
		}

		@Override
		public boolean getDoTimeFreezing()
		{
			// TODO Auto-generated method stub
			return false;
		}

		@Override
		public boolean getDoFootSlipCompensation()
		{
			// TODO Auto-generated method stub
			return false;
		}

		@Override
		public double getAlphaDeltaFootPositionForFootslipCompensation()
		{
			return 0.65;
		}

      @Override
      public double getEntryCMPInsideOffset()
      {
         return 0.006;
      }

      @Override
      public double getExitCMPInsideOffset()
      {
         return 0.006;
      }

      @Override
      public double getEntryCMPForwardOffset()
      {
         return 0.0;
      }

      @Override
      public double getExitCMPForwardOffset()
      {
         return 0.0;
      }
      
		@Override
		public double getMaxAllowedErrorWithoutPartialTimeFreeze()
		{
			return 0.03;
		}

	   @Override
	   public boolean useTerribleHackToReduceICPVelocityAtTheEndOfTransfer()
	   {
	      return true;
	   }

	   @Override
	   public boolean useNewICPPlanner()
	   {
	      return false;
	   }

	   @Override
	   public boolean useTwoCMPsPerSupport()
	   {
	      return false;
	   }

	   @Override
	   public double getTimeSpentOnExitCMPInPercentOfStepTime()
	   {
	      return 0.50;
	   }

	   @Override
	   public double getMaxEntryCMPForwardOffset()
	   {
	      return 0.05;
	   }

	   @Override
	   public double getMinEntryCMPForwardOffset()
	   {
	      return -0.02;
	   }

	   @Override
	   public double getMaxExitCMPForwardOffset()
	   {
	      return 0.05;
	   }

	   @Override
	   public double getMinExitCMPForwardOffset()
	   {
	      return -0.02;
	   }

	   @Override
	   public double getCMPSafeDistanceAwayFromSupportEdges()
	   {
	      return 0.03;
	   }

      @Override
      public double getMaxDurationForSmoothingEntryToExitCMPSwitch()
      {
         return 0.5;
      }

      /** {@inheritDoc} */
      @Override
      public double getStepLengthToCMPOffsetFactor()
      {
         return 1.0 / 3.0;
      }

      /** {@inheritDoc} */
      @Override
      public boolean useExitCMPOnToesForSteppingDown()
      {
         return false;
      }

      /** {@inheritDoc} */
      @Override
      public double getStepLengthThresholdForExitCMPOnToesWhenSteppingDown()
      {
         return 0.15;
      }

      /** {@inheritDoc} */
      @Override
      public double getStepHeightThresholdForExitCMPOnToesWhenSteppingDown()
      {
         return 0.10;
      }

      /** {@inheritDoc} */
      @Override
      public double getCMPSafeDistanceAwayFromToesWhenSteppingDown()
      {
         return 0.0;
      }
	};

	private double singleSupportDuration = testICPPlannerParams.getSingleSupportDuration();
	private double doubleSupportDuration = testICPPlannerParams.getDoubleSupportDuration();
	private double doubleSupportInitialTransferDuration = testICPPlannerParams.getDoubleSupportInitialTransferDuration();
	private int numberOfStepsInStepList = 7;
	private int maxNumberOfConsideredFootsteps = testICPPlannerParams.getNumberOfFootstepsToConsider();
	private NewInstantaneousCapturePointPlannerWithSmoother icpPlanner;

	private YoFrameLineSegment2d icpVelocityLineSegment = null;

	private double scsPlaybackRate = 0.5;

	private double scsPlaybackDesiredFrameRate = 0.001;

	@After
	public void removeVisualizersAfterTest()
	{
		if (scs != null)
		{
			scs.closeAndDispose();
		}

		visualize = false;

		pointAndLinePlotter = null;
		yoGraphicsListRegistry = null;
		scs = null;
		timeYoVariable = null;
		registry = null;

		icpArrowTip = null;
		icpPositionYoFramePoint = null;

		icpVelocityYoFramePoint = null;

		cmpPositionYoFramePoint = null;

		footstepYoFramePoints = null;

		icpFootCenterCornerPointsViz = null;

		constantCoPsViz = null;

		doubleSupportStartICPYoFramePoint = null;
		doubleSupportEndICPYoFramePoint = null;

		icpVelocityLineSegment = null;
	}

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
	public void testCapturePointPlannerWithSmootherNoPushOrCancelPlan()
	{
		boolean cancelPlan = false;
		visualize = false;

		icpPlanner = new NewInstantaneousCapturePointPlannerWithSmoother(testICPPlannerParams, registry, yoGraphicsListRegistry);

		RobotSide stepSide = RobotSide.LEFT;
		double stepLength = 0.3;
		double halfStepWidth = 0.1;
		boolean startSquaredUp = true;

		double comHeight = 1.0;
		double gravitationalAcceleration = 9.81;

		double omega0 = Math.sqrt(gravitationalAcceleration / comHeight);

		FrameTupleArrayList<FramePoint> footLocations = createABunchOfUniformWalkingSteps(stepSide, startSquaredUp, numberOfStepsInStepList,
				stepLength, halfStepWidth);

		FramePoint initialICPPosition = new FramePoint(ReferenceFrame.getWorldFrame());
		FramePoint singleSupportStartICP = new FramePoint(ReferenceFrame.getWorldFrame());
		FrameVector initialICPVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
		FrameVector initialICPAcceleration = new FrameVector(ReferenceFrame.getWorldFrame());

		FramePoint icpPosition = new FramePoint(ReferenceFrame.getWorldFrame());
		FrameVector icpVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
		FrameVector icpAcceleration = new FrameVector(ReferenceFrame.getWorldFrame());

		FramePoint cmpPosition = new FramePoint(ReferenceFrame.getWorldFrame());

		double initialTime = 0.0;

		initialICPPosition.set(footLocations.get(0));
		initialICPPosition.add(footLocations.get(1));
		initialICPPosition.scale(0.5);

		icpPlanner.setOmega0(omega0);
		icpPlanner.initializeDoubleSupport(initialICPPosition, initialICPVelocity, initialTime, footLocations);

		icpPlanner.packDesiredCapturePointPositionVelocityAndAcceleration(icpPosition, icpVelocity, icpAcceleration, initialTime);

		JUnitTools.assertPoint3dEquals("", initialICPPosition.getPointCopy(), icpPosition.getPointCopy(), 1e-10);
		JUnitTools.assertVector3dEquals("", initialICPVelocity.getVectorCopy(), icpVelocity.getVectorCopy(), 1e-10);

		simulateForwardAndCheckDoubleSupport(footLocations, icpPosition, icpVelocity, icpAcceleration, cmpPosition, icpPlanner,
				doubleSupportInitialTransferDuration, initialTime, deltaT, omega0, initialICPPosition, false);

		ArrayList<YoFramePoint> cornerPoints = icpPlanner.getCapturePointCornerPoints();
		ArrayList<YoFramePoint> constantCMPs = icpPlanner.getConstantCentroidalMomentumPivots();

		assertEquals(cmpPosition.getX(), constantCMPs.get(1).getX(), 1e-4);
		assertEquals(cmpPosition.getY(), constantCMPs.get(1).getY(), 1e-4);
		JUnitTools.assertPoint3dEquals("", cornerPoints.get(1).getPoint3dCopy(), icpPosition.getPointCopy(), 1e-4);

		initialTime = initialTime + doubleSupportInitialTransferDuration;

		footLocations.remove(0);

		icpPlanner.initializeSingleSupport(initialTime, footLocations);

		icpPlanner.packDesiredCapturePointPositionVelocityAndAcceleration(initialICPPosition, initialICPVelocity, initialICPAcceleration,
				initialTime);

		simulateForwardAndCheckSingleSupport(icpPosition, icpVelocity, icpAcceleration, cmpPosition, icpPlanner, singleSupportDuration,
				initialTime, omega0, initialICPPosition, footLocations, false);

		cornerPoints = icpPlanner.getCapturePointCornerPoints();
		constantCMPs = icpPlanner.getConstantCentroidalMomentumPivots();

		FramePoint expectedICP = new FramePoint(ReferenceFrame.getWorldFrame());
		expectedICP.setX(constantCMPs.get(0).getX() * (1 - Math.exp(omega0 * singleSupportDuration)) + cornerPoints.get(0).getX()
				* Math.exp(omega0 * singleSupportDuration));
		expectedICP.setY(constantCMPs.get(0).getY() * (1 - Math.exp(omega0 * singleSupportDuration)) + cornerPoints.get(0).getY()
				* Math.exp(omega0 * singleSupportDuration));

		assertEquals(expectedICP.getX(), icpPosition.getX(), 1e-4);
		assertEquals(expectedICP.getY(), icpPosition.getY(), 1e-4);
	}

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
	public void testCapturePointPlannerWithCancelPlan()
	{
		boolean testPush = false;
		boolean cancelPlan = true;
		visualize = false;

		icpPlanner = new NewInstantaneousCapturePointPlannerWithSmoother(testICPPlannerParams, registry, yoGraphicsListRegistry);

		RobotSide stepSide = RobotSide.LEFT;
		double stepLength = 0.3;
		double halfStepWidth = 0.1;
		boolean startSquaredUp = true;

		double comHeight = 1.0;
		double gravitationalAcceleration = 9.81;

		double omega0 = Math.sqrt(gravitationalAcceleration / comHeight);

		FrameTupleArrayList<FramePoint> footLocations = createABunchOfUniformWalkingSteps(stepSide, startSquaredUp, numberOfStepsInStepList,
				stepLength, halfStepWidth);

		FramePoint initialICPPosition = new FramePoint(ReferenceFrame.getWorldFrame());
		FrameVector initialICPVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
		FrameVector initialICPAcceleration = new FrameVector(ReferenceFrame.getWorldFrame());

		FramePoint icpPosition = new FramePoint(ReferenceFrame.getWorldFrame());
		FrameVector icpVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
		FrameVector icpAcceleration = new FrameVector(ReferenceFrame.getWorldFrame());

		FramePoint cmpPosition = new FramePoint(ReferenceFrame.getWorldFrame());

		double initialTime = 0.0;

		initialICPPosition.set(footLocations.get(0));
		initialICPPosition.add(footLocations.get(1));
		initialICPPosition.scale(0.5);

		icpPlanner.setOmega0(omega0);
		icpPlanner.initializeDoubleSupport(initialICPPosition, initialICPVelocity, initialTime, footLocations);

		icpPlanner.packDesiredCapturePointPositionVelocityAndAcceleration(icpPosition, icpVelocity, icpAcceleration, initialTime);

		JUnitTools.assertPoint3dEquals("", initialICPPosition.getPointCopy(), icpPosition.getPointCopy(), 1e-10);
		JUnitTools.assertVector3dEquals("", initialICPVelocity.getVectorCopy(), icpVelocity.getVectorCopy(), 1e-10);

		simulateForwardAndCheckDoubleSupport(footLocations, icpPosition, icpVelocity, icpAcceleration, cmpPosition, icpPlanner,
				doubleSupportInitialTransferDuration, initialTime, deltaT, omega0, initialICPPosition, false);

		ArrayList<YoFramePoint> cornerPoints = icpPlanner.getCapturePointCornerPoints();
		ArrayList<YoFramePoint> constantCMPs = icpPlanner.getConstantCentroidalMomentumPivots();

		assertEquals(cmpPosition.getX(), constantCMPs.get(1).getX(), 1e-4);
		assertEquals(cmpPosition.getY(), constantCMPs.get(1).getY(), 1e-4);
		JUnitTools.assertPoint3dEquals("", cornerPoints.get(1).getPoint3dCopy(), icpPosition.getPointCopy(), 1e-4);

		initialTime = initialTime + doubleSupportInitialTransferDuration;

		footLocations.remove(0);

		icpPlanner.initializeSingleSupport(initialTime, footLocations);

		icpPlanner.packDesiredCapturePointPositionVelocityAndAcceleration(initialICPPosition, initialICPVelocity, initialICPAcceleration,
				initialTime);

		simulateForwardAndCheckSingleSupport(icpPosition, icpVelocity, icpAcceleration, cmpPosition, icpPlanner, singleSupportDuration,
				initialTime, omega0, initialICPPosition, footLocations, false);

		initialTime = initialTime + singleSupportDuration;

		icpPlanner.initializeDoubleSupport(icpPosition, icpVelocity, initialTime, footLocations);
		icpPlanner.packDesiredCapturePointPositionVelocityAndAcceleration(initialICPPosition, initialICPVelocity, initialICPAcceleration,
				initialTime);

		simulateForwardAndCheckDoubleSupport(footLocations, icpPosition, icpVelocity, icpAcceleration, cmpPosition, icpPlanner,
				doubleSupportDuration, initialTime, deltaT, omega0, initialICPPosition, cancelPlan);

		FramePoint finalPosition = footLocations.get(0);
		finalPosition.add(footLocations.get(1));
		finalPosition.scale(0.5);
		assertEquals(finalPosition.getX(), cmpPosition.getX(), 1e-4);
		assertEquals(finalPosition.getY(), cmpPosition.getY(), 1e-4);

		assertEquals(finalPosition.getX(), icpPosition.getX(), 1e-4);
		assertEquals(finalPosition.getY(), icpPosition.getY(), 1e-4);
	}

	@Ignore
	public void VisualizePlanner()
	{
		boolean cancelPlan = false;
		visualize = true;

		icpPlanner = new NewInstantaneousCapturePointPlannerWithSmoother(testICPPlannerParams, registry, yoGraphicsListRegistry);

		icpPlanner.setDoubleSupportSplitFraction(0.5);
		createVisualizers(maxNumberOfConsideredFootsteps);

		RobotSide stepSide = RobotSide.LEFT;
		double stepLength = 0.3;
		double halfStepWidth = 0.1;
		boolean startSquaredUp = true;

		double comHeight = 1.0;
		double gravitationalAcceleration = 9.81;

		double omega0 = Math.sqrt(gravitationalAcceleration / comHeight);

		FrameTupleArrayList<FramePoint> footLocations = createABunchOfUniformWalkingSteps(stepSide, startSquaredUp, numberOfStepsInStepList,
				stepLength, halfStepWidth);

		FramePoint initialICPPosition = new FramePoint(ReferenceFrame.getWorldFrame());
		FramePoint singleSupportStartICP = new FramePoint(ReferenceFrame.getWorldFrame());
		FrameVector initialICPVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
		FrameVector initialICPAcceleration = new FrameVector(ReferenceFrame.getWorldFrame());

		FramePoint icpPosition = new FramePoint(ReferenceFrame.getWorldFrame());
		FrameVector icpVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
		FrameVector icpAcceleration = new FrameVector(ReferenceFrame.getWorldFrame());

		FramePoint cmpPosition = new FramePoint(ReferenceFrame.getWorldFrame());

		double initialTime = 0.0;

		initialICPPosition.set(footLocations.get(0));
		initialICPPosition.add(footLocations.get(1));
		initialICPPosition.scale(0.5);

		icpPlanner.setOmega0(omega0);
		icpPlanner.initializeDoubleSupport(initialICPPosition, initialICPVelocity, initialTime, footLocations);

		icpPlanner.getSingleSupportInitialCapturePointPosition(singleSupportStartICP);
		singleSupportInitialICPPosition.set(singleSupportStartICP);

		icpPlanner.packDesiredCapturePointPositionVelocityAndAcceleration(initialICPPosition, initialICPVelocity, initialICPAcceleration,
				initialTime);

		if (visualize)
		{
			for (int i = 0; i < Math.min(footLocations.size(), footstepYoFramePoints.size()); i++)
			{
				footstepYoFramePoints.get(i).set(footLocations.get(i));
			}

			for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
			{
				constantCoPsViz.get(i).set(icpPlanner.getConstantCentroidalMomentumPivots().get(i));
			}

			for (int i = 0; i < maxNumberOfConsideredFootsteps - 1; i++)
			{
				icpFootCenterCornerPointsViz.get(i).set(icpPlanner.getCapturePointCornerPoints().get(i));
			}
		}

		simulateForwardAndCheckDoubleSupport(footLocations, icpPosition, icpVelocity, icpAcceleration, cmpPosition, icpPlanner,
				doubleSupportInitialTransferDuration, initialTime, deltaT, omega0, initialICPPosition, false);

		initialTime = initialTime + doubleSupportInitialTransferDuration;

		footLocations.remove(0);

		for (int i = 0; i < Math.min(footLocations.size(), footstepYoFramePoints.size()); i++)
		{
			footstepYoFramePoints.get(i).set(footLocations.get(i));
		}

		while (footLocations.size() >= 2)
		{
			icpPlanner.initializeSingleSupport(initialTime, footLocations);

			if (visualize)
			{
				for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
				{
					constantCoPsViz.get(i).set(icpPlanner.getConstantCentroidalMomentumPivots().get(i).getFramePointCopy());
				}

				for (int i = 0; i < maxNumberOfConsideredFootsteps - 1; i++)
				{
					icpFootCenterCornerPointsViz.get(i).set(icpPlanner.getCapturePointCornerPoints().get(i).getFramePointCopy());
				}
			}

			icpPlanner.packDesiredCapturePointPositionVelocityAndAcceleration(initialICPPosition, initialICPVelocity,
					initialICPAcceleration, initialTime);

			simulateForwardAndCheckSingleSupport(icpPosition, icpVelocity, icpAcceleration, cmpPosition, icpPlanner, singleSupportDuration,
					initialTime, omega0, initialICPPosition, footLocations, false);

			singleSupportFinalICPPosition.set(icpPosition);

			initialTime = initialTime + singleSupportDuration;

			icpPlanner.initializeDoubleSupport(icpPosition, icpVelocity, initialTime, footLocations);
			icpPlanner.packDesiredCapturePointPositionVelocityAndAcceleration(initialICPPosition, initialICPVelocity,
					initialICPAcceleration, initialTime);

			icpPlanner.getSingleSupportInitialCapturePointPosition(singleSupportStartICP);
			singleSupportInitialICPPosition.set(singleSupportStartICP);

			if (visualize)
			{
				for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
				{
					constantCoPsViz.get(i).set(icpPlanner.getConstantCentroidalMomentumPivots().get(i).getFramePointCopy());
				}

				for (int i = 0; i < maxNumberOfConsideredFootsteps - 1; i++)
				{
					icpFootCenterCornerPointsViz.get(i).set(icpPlanner.getCapturePointCornerPoints().get(i).getFramePointCopy());
				}
			}

			simulateForwardAndCheckDoubleSupport(footLocations, icpPosition, icpVelocity, icpAcceleration, cmpPosition, icpPlanner,
					doubleSupportDuration, initialTime, deltaT, omega0, initialICPPosition, cancelPlan);
			if (cancelPlan)
			{
				break;
			}

			initialTime = initialTime + doubleSupportDuration;

			footLocations.remove(0);

			for (int i = 0; i < Math.min(footLocations.size(), footstepYoFramePoints.size()); i++)
			{
				footstepYoFramePoints.get(i).set(footLocations.get(i));
			}
		}

		if (visualize)
		{
			pointAndLinePlotter.addPointsAndLinesToSCS(scs);

			scs.startOnAThread();
			ThreadTools.sleepForever();
		}
	}

	private FrameTupleArrayList<FramePoint> createABunchOfUniformWalkingSteps(RobotSide stepSide, boolean startSquaredUp,
			int numberOfStepsInStepList, double stepLength, double halfStepWidth)
	{
	   FrameTupleArrayList<FramePoint> footLocations = FrameTupleArrayList.createFramePointArrayList();

		double height = 0.5;

		if (startSquaredUp)
		{
			FramePoint firstStepLocation = footLocations.add();
			firstStepLocation.set(0, stepSide.negateIfRightSide(halfStepWidth), height);

			stepSide = stepSide.getOppositeSide();
		}

		for (int i = 0; i < numberOfStepsInStepList; i++)
		{
			FramePoint stepLocation = footLocations.add();
			stepLocation.set(i * stepLength, stepSide.negateIfRightSide(halfStepWidth), height);

			stepSide = stepSide.getOppositeSide();
		}

		return footLocations;
	}

	private void createVisualizers(int maxNumberOfConsideredFootsteps)
	{
		ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

		Robot robot = new Robot("TestRobot");
		scs = new SimulationConstructionSet(robot);

		scs.setDT(deltaT, 1);
		scs.changeBufferSize(16000);
		scs.setPlaybackRealTimeRate(scsPlaybackRate);
		scs.setPlaybackDesiredFrameRate(scsPlaybackDesiredFrameRate);

		robot.getRobotsYoVariableRegistry().addChild(registry);
		timeYoVariable = robot.getYoTime();

		pointAndLinePlotter.createAndShowOverheadPlotterInSCS(scs);

		icpArrowTip = new YoFramePoint("icpVelocityTip", "", worldFrame, registry);
		singleSupportInitialICPPosition = new YoFramePoint("flegm", "", worldFrame, registry);
		singleSupportFinalICPPosition = new YoFramePoint("flegm2", "", worldFrame, registry);
		icpVelocityLineSegment = new YoFrameLineSegment2d("icpVelocityForViz", "", worldFrame, registry);
		icpPositionYoFramePoint = new YoFramePoint("icpPositionForViz", "", worldFrame, registry);
		icpPositionYoFramePoint.setZ(0.5);
		icpVelocityYoFramePoint = new YoFramePoint("icpVelocityForViz", "", worldFrame, registry);
		cmpPositionYoFramePoint = new YoFramePoint("cmpPositionForViz", "", worldFrame, registry);
		cmpPositionYoFramePoint.setZ(0.5);

		doubleSupportStartICPYoFramePoint = new YoFramePoint("doubleSupportICPStart", "", worldFrame, registry);
		doubleSupportEndICPYoFramePoint = new YoFramePoint("doubleSupportICPEnd", "", worldFrame, registry);

		footstepYoFramePoints = new ArrayList<YoFramePoint>();
		icpFootCenterCornerPointsViz = new ArrayList<YoFramePoint>();
		constantCoPsViz = new ArrayList<YoFramePoint>();

		for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
		{
			YoFramePoint footstepYoFramePoint = pointAndLinePlotter.plotPoint3d("footstep" + i, new Point3d(), YoAppearance.Black(), 0.009);
			footstepYoFramePoints.add(footstepYoFramePoint);

		}

		for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
		{
			YoFramePoint constantCoPPoint = pointAndLinePlotter
					.plotPoint3d("constantCoPsViz" + i, new Point3d(), YoAppearance.Red(), 0.004);
			constantCoPsViz.add(constantCoPPoint);
		}

		for (int i = 0; i < maxNumberOfConsideredFootsteps - 1; i++)
		{
			YoFramePoint cornerICPFramePoint = pointAndLinePlotter.plotPoint3d("icpCornerPoint" + i, new Point3d(), YoAppearance.Green(),
					0.003);
			icpFootCenterCornerPointsViz.add(cornerICPFramePoint);
		}

		pointAndLinePlotter.plotYoFramePoint("icpPosition", icpPositionYoFramePoint, YoAppearance.Gold(), 0.005);
		pointAndLinePlotter.plotYoFramePoint("icpFlegmPosition", singleSupportInitialICPPosition, YoAppearance.Crimson(), 0.004);
		pointAndLinePlotter.plotYoFramePoint("icpFlegmPosition2", singleSupportFinalICPPosition, YoAppearance.Crimson(), 0.004);
		pointAndLinePlotter.plotYoFramePoint("cmpPosition", cmpPositionYoFramePoint, YoAppearance.Magenta(), 0.005);
		pointAndLinePlotter.plotLineSegment("icpVelocity", icpVelocityLineSegment, Color.gray);

		pointAndLinePlotter.plotYoFramePoint("doubleSupportICPStart", doubleSupportStartICPYoFramePoint, YoAppearance.Cyan(), 0.003);
		pointAndLinePlotter.plotYoFramePoint("doubleSupportICPEnd", doubleSupportEndICPYoFramePoint, YoAppearance.Cyan(), 0.004);

		yoGraphicsListRegistry = pointAndLinePlotter.getDynamicGraphicObjectsListRegistry();
	}

	private void simulateForwardAndCheckSingleSupport(FramePoint icpPositionToPack, FrameVector icpVelocityToPack,
			FrameVector icpAccelerationToPack, FramePoint cmpPositionToPack, NewInstantaneousCapturePointPlannerWithSmoother icpPlanner,
			double singleSupportDuration, double initialTime, double omega0, FramePoint initialICPPosition,
			FrameTupleArrayList<FramePoint> footstepList, boolean cancelPlan)
	{
		for (double time = initialTime + deltaT; time <= initialTime + singleSupportDuration; time = time + deltaT)
		{
			icpPlanner.packDesiredCapturePointPositionVelocityAndAcceleration(icpPositionToPack, icpVelocityToPack, icpAccelerationToPack,
					time);
			icpPlanner.packDesiredCentroidalMomentumPivotPosition(cmpPositionToPack);

			if (visualize)
			{
				visualizeICPAndECMP(icpPositionToPack, icpVelocityToPack, cmpPositionToPack, time);
			}

			if (cancelPlan && time > initialTime + 0.10)
			{
				icpPlanner.cancelPlan(time, footstepList);
				cancelPlan = false;
				initialTime = time;

				if (visualize)
				{
					for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
					{
						constantCoPsViz.get(i).set(icpPlanner.getConstantCentroidalMomentumPivots().get(i));
					}

					for (int i = 0; i < maxNumberOfConsideredFootsteps - 1; i++)
					{
						icpFootCenterCornerPointsViz.get(i).set(icpPlanner.getCapturePointCornerPoints().get(i));
					}
				}
			}

			initialICPPosition.set(icpPositionToPack);
		}
	}

	private void simulateForwardAndCheckDoubleSupport(FrameTupleArrayList<FramePoint> footstepList, FramePoint icpPositionToPack,
			FrameVector icpVelocityToPack, FrameVector icpAccelerationToPack, FramePoint cmpPositionToPack,
			NewInstantaneousCapturePointPlannerWithSmoother icpPlanner, double doubleSupportDuration, double initialTime, double deltaT,
			double omega0, FramePoint initialICPPosition, boolean cancelPlan)
	{
		for (double time = initialTime + deltaT; time <= initialTime + doubleSupportDuration; time = time + deltaT)
		{
			icpPlanner.packDesiredCapturePointPositionVelocityAndAcceleration(icpPositionToPack, icpVelocityToPack, icpAccelerationToPack,
					time);
			icpPlanner.packDesiredCentroidalMomentumPivotPosition(cmpPositionToPack);

			if (visualize)
			{
				visualizeICPAndECMP(icpPositionToPack, icpVelocityToPack, cmpPositionToPack, time);
			}

			if (cancelPlan && time > initialTime + 0.10)
			{
				icpPlanner.cancelPlan(time, footstepList);
				cancelPlan = false;
				initialTime = time;

				if (visualize)
				{
					for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
					{
						constantCoPsViz.get(i).set(icpPlanner.getConstantCentroidalMomentumPivots().get(i));
					}

					for (int i = 0; i < maxNumberOfConsideredFootsteps - 1; i++)
					{
						icpFootCenterCornerPointsViz.get(i).set(icpPlanner.getCapturePointCornerPoints().get(i));
					}
				}
			}

			initialICPPosition.set(icpPositionToPack);
		}
	}

	private void visualizeICPAndECMP(FramePoint icpPosition, FrameVector icpVelocity, FramePoint cmpPosition, double time)
	{
		icpPositionYoFramePoint.set(icpPosition);
		icpVelocityYoFramePoint.set(icpVelocity);
		cmpPositionYoFramePoint.set(cmpPosition);

		PointAndLinePlotter.setEndPointGivenStartAndAdditionalVector(icpArrowTip, icpPosition.getPointCopy(), icpVelocity.getVectorCopy(),
				0.5);
		Point2d icpPosition2d = new Point2d(icpPosition.getX(), icpPosition.getY());
		PointAndLinePlotter.setLineSegmentBasedOnStartAndEndFramePoints(icpVelocityLineSegment, icpPosition2d, icpArrowTip
				.getFramePoint2dCopy().getPointCopy());

		timeYoVariable.set(time);
		scs.tickAndUpdate();
	}

	private void updateFootstepsFromPush(ArrayList<FramePoint> footstepList)
	{
		double tmpx = random.nextDouble() * 0.1;
		double tmpy = random.nextDouble() * 0.05;

		for (int i = 1; i < footstepList.size() - 1; i++)
		{
			footstepList.get(i).setX(footstepList.get(i).getX() + tmpx);
			footstepList.get(i).setY(footstepList.get(i).getY() + tmpy);
		}

		if (visualize)
		{
			for (int i = 0; i < Math.min(footstepList.size(), footstepYoFramePoints.size()); i++)
			{
				footstepYoFramePoints.get(i).set(footstepList.get(i));
			}
		}
	}
}
