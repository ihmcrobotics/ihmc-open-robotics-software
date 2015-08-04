package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import org.junit.After;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.PointAndLinePlotter;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.utilities.code.agileTesting.BambooPlanType;
import us.ihmc.robotics.lists.FrameTupleArrayList;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFrameLineSegment2d;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;

@BambooPlan(planType = {BambooPlanType.UI})
public class NewInstantaneousCapturePointPlannerWithTimeFreezerAndFootSlipCompensationTest
{
	private YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

	private boolean visualize = false;
	private final Random random = new Random();

	private PointAndLinePlotter pointAndLinePlotter = new PointAndLinePlotter(registry);
	private YoGraphicsListRegistry yoGraphicsListRegistry = null;
	private final ArrayList<ArrayList<YoFrameLineSegment2d>> listOfFootPolygons = new ArrayList<ArrayList<YoFrameLineSegment2d>>();
	private SimulationConstructionSet scs = null;
	private DoubleYoVariable timeYoVariable = null;
	private double sineAmplitude = 0.04;
	private double sineFrequency = 5;

	private double footLength = 0.2;
	private double footWidth = 0.1;

	YoFramePoint tmpPreviousICPPosition = new YoFramePoint("PreviousICPPosition", ReferenceFrame.getWorldFrame(), registry);

	private YoFramePoint icpArrowTip = null;
	private YoFramePoint icpPositionYoFramePoint = null;
	private YoFramePoint singleSupportInitialICPPosition = null;
	private YoFramePoint singleSupportFinalICPPosition = null;
	private YoFramePoint icpVelocityYoFramePoint = null;
	private YoFramePoint cmpPositionYoFramePoint = null;
	private ArrayList<YoFrameLineSegment2d> footLineSegments1 = new ArrayList<YoFrameLineSegment2d>();
	private ArrayList<YoFrameLineSegment2d> footLineSegments2 = new ArrayList<YoFrameLineSegment2d>();

	FramePoint actualICPPosition = new FramePoint(ReferenceFrame.getWorldFrame());

	private YoFramePoint actualICPPositionYoFramePoint = null;

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
			return 4;
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
			return 0.5;
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
			return false;
		}

		@Override
		public boolean getDoFootSlipCompensation()
		{
			return true;
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
         return 0;
      }

      @Override
      public double getExitCMPForwardOffset()
      {
         return 0;
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
	private final RobotSide robotSide = RobotSide.RIGHT;
	private NewInstantaneousCapturePointPlannerWithTimeFreezerAndFootSlipCompensation icpPlanner;

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

	@EstimatedDuration
	@Test(timeout=300000)
	public void visualizePlanner()
	{
		boolean testPush = true;
		boolean cancelPlanInDoubleSupport = false;
		boolean cancelPlanInSingleSupport = false;
		boolean breakAfterDoubleSupportPhase = cancelPlanInDoubleSupport || cancelPlanInSingleSupport;
		boolean testFootslipCompensation = false;
		for (int i = 0; i < 4; i++)
		{
			YoFrameLineSegment2d line1 = new YoFrameLineSegment2d("line" + i, "", ReferenceFrame.getWorldFrame(), registry);
			YoFrameLineSegment2d line2 = new YoFrameLineSegment2d("line" + i + i, "", ReferenceFrame.getWorldFrame(), registry);
			footLineSegments1.add(line1);
			footLineSegments2.add(line2);
		}

		listOfFootPolygons.add(footLineSegments1);
		listOfFootPolygons.add(footLineSegments2);

		icpPlanner = new NewInstantaneousCapturePointPlannerWithTimeFreezerAndFootSlipCompensation(testICPPlannerParams, registry,
				yoGraphicsListRegistry);

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

		FramePoint icpPosition = new FramePoint(ReferenceFrame.getWorldFrame());
		FrameVector icpVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
		FrameVector icpAcceleration = new FrameVector(ReferenceFrame.getWorldFrame());

		FramePoint cmpPosition = new FramePoint(ReferenceFrame.getWorldFrame());

		double initialTime = 0.0;

		initialICPPosition.set(footLocations.get(0));
		initialICPPosition.add(footLocations.get(1));
		initialICPPosition.scale(0.5);

		actualICPPosition.set(initialICPPosition);

		icpPlanner.setOmega0(omega0);
		icpPlanner.initializeDoubleSupport(initialICPPosition, initialICPVelocity, initialTime, footLocations,robotSide,footLocations.get(1));

		icpPlanner.getSingleSupportInitialCapturePointPosition(singleSupportStartICP);
		singleSupportInitialICPPosition.set(singleSupportStartICP);

		icpPlanner.packDesiredCapturePointPositionAndVelocity(initialICPPosition, initialICPVelocity, initialTime, actualICPPosition, footLocations.get(1));

		if (visualize)
		{
			updatePointsVis(footLocations);
		}

		simulateForwardAndCheckDoubleSupport(footLocations, icpPosition, icpVelocity, icpAcceleration, cmpPosition, icpPlanner,
				doubleSupportInitialTransferDuration, initialTime, deltaT, omega0, initialICPPosition, false, testFootslipCompensation);

		initialTime = initialTime + doubleSupportInitialTransferDuration;

		footLocations.remove(0);

		updatePointsVis(footLocations);

		while (footLocations.size() >= 2)
		{
			icpPlanner.initializeSingleSupport(initialTime, footLocations);
			icpPlanner.getSingleSupportInitialCapturePointPosition(singleSupportStartICP);
			singleSupportInitialICPPosition.set(singleSupportStartICP);

			if (visualize)
			{
				updatePointsVis(footLocations);
			}

			icpPlanner.packDesiredCapturePointPositionAndVelocity(initialICPPosition, initialICPVelocity, initialTime, actualICPPosition,
					null);

			simulateForwardAndCheckSingleSupport(icpPosition, icpVelocity, icpAcceleration, cmpPosition, icpPlanner, singleSupportDuration,
					initialTime, omega0, initialICPPosition, footLocations, testPush, cancelPlanInSingleSupport);
			if (testPush)
			{
				testPush = false;
			}

			singleSupportFinalICPPosition.set(icpPosition);

			initialTime = initialTime + singleSupportDuration;

			icpPlanner.initializeDoubleSupport(icpPosition, icpVelocity, initialTime, footLocations,robotSide.getOppositeSide(),footLocations.get(1));
			icpPlanner.packDesiredCapturePointPositionAndVelocity(initialICPPosition, initialICPVelocity, initialTime, actualICPPosition,
					footLocations.get(1));

			icpPlanner.getSingleSupportInitialCapturePointPosition(singleSupportStartICP);
			singleSupportInitialICPPosition.set(singleSupportStartICP);

			if (visualize)
			{
				updatePointsVis(footLocations);
			}

			simulateForwardAndCheckDoubleSupport(footLocations, icpPosition, icpVelocity, icpAcceleration, cmpPosition, icpPlanner,
					doubleSupportDuration, initialTime, deltaT, omega0, initialICPPosition, cancelPlanInDoubleSupport,
					testFootslipCompensation);
			if (breakAfterDoubleSupportPhase)
			{
				break;
			}

			initialTime = initialTime + doubleSupportDuration;

			footLocations.remove(0);

			updatePointsVis(footLocations);
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
		actualICPPositionYoFramePoint = new YoFramePoint("icpActualPositionForViz", worldFrame, registry);
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
		pointAndLinePlotter.plotYoFramePoint("blahblah", actualICPPositionYoFramePoint, YoAppearance.AliceBlue(), 0.005);
		pointAndLinePlotter.plotLineSegments("footstep1", listOfFootPolygons.get(0), Color.black);
		pointAndLinePlotter.plotLineSegments("footstep2", listOfFootPolygons.get(1), Color.black);
		pointAndLinePlotter.plotLineSegment("icpVelocity", icpVelocityLineSegment, Color.gray);

		pointAndLinePlotter.plotYoFramePoint("doubleSupportICPStart", doubleSupportStartICPYoFramePoint, YoAppearance.Cyan(), 0.003);
		pointAndLinePlotter.plotYoFramePoint("doubleSupportICPEnd", doubleSupportEndICPYoFramePoint, YoAppearance.Cyan(), 0.004);

		yoGraphicsListRegistry = pointAndLinePlotter.getDynamicGraphicObjectsListRegistry();
	}

	private void simulateForwardAndCheckSingleSupport(FramePoint icpPositionToPack, FrameVector icpVelocityToPack,
			FrameVector icpAccelerationToPack, FramePoint cmpPositionToPack,
			NewInstantaneousCapturePointPlannerWithTimeFreezerAndFootSlipCompensation icpPlanner, double singleSupportDuration,
			double initialTime, double omega0, FramePoint initialICPPosition, FrameTupleArrayList<FramePoint> footstepList, boolean testPush,
			boolean cancelPlan)
	{
		double time = initialTime + deltaT;
		while (!icpPlanner.isDone(time))
		{
			icpPlanner.packDesiredCapturePointPositionAndVelocity(icpPositionToPack, icpVelocityToPack, time, actualICPPosition,
					footstepList.get(1));
			icpPlanner.packDesiredCentroidalMomentumPivotPosition(cmpPositionToPack);

			actualICPPosition.set(icpPositionToPack);
			actualICPPosition.setX(actualICPPosition.getX() + sineAmplitude * Math.sin(sineFrequency * (time - initialTime)));
			actualICPPosition.setY(actualICPPosition.getY() + sineAmplitude * Math.sin(sineFrequency * (time - initialTime)));
			actualICPPositionYoFramePoint.set(actualICPPosition);

			if (icpPlanner.getIsTimeBeingFrozen())
			{
				doubleSupportDuration = doubleSupportDuration + deltaT;
			}

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
					updatePointsVis(footstepList);
				}
			}

			initialICPPosition.set(icpPositionToPack);

			if (testPush)
			{
				if (time > initialTime + 0.2)
				{
					updateFootstepsFromPush(footstepList);

				   icpPlanner.updatePlanForSingleSupportDisturbances(time, footstepList, actualICPPosition);
					if (visualize)
					{
						updatePointsVis(footstepList);
					}
					testPush = false;
				}
			}

			time += deltaT;
		}
	}

	private void simulateForwardAndCheckDoubleSupport(FrameTupleArrayList<FramePoint> footstepList, FramePoint icpPositionToPack,
			FrameVector icpVelocityToPack, FrameVector icpAccelerationToPack, FramePoint cmpPositionToPack,
			NewInstantaneousCapturePointPlannerWithTimeFreezerAndFootSlipCompensation icpPlanner, double doubleSupportDuration,
			double initialTime, double deltaT, double omega0, FramePoint initialICPPosition, boolean cancelPlan,
			boolean testFootslipCompensation)
	{
		FramePoint transferToFootstep = footstepList.get(1);
		double time = initialTime + deltaT;
		while (!icpPlanner.isDone(time))
		{
			icpPlanner.packDesiredCapturePointPositionAndVelocity(icpPositionToPack, icpVelocityToPack, time, actualICPPosition,
					footstepList.get(1));
			icpPlanner.packDesiredCentroidalMomentumPivotPosition(cmpPositionToPack);

			actualICPPosition.set(icpPositionToPack);
			actualICPPosition.setX(actualICPPosition.getX() - sineAmplitude * Math.sin(sineFrequency * (time - initialTime)));
			actualICPPosition.setY(actualICPPosition.getY() - sineAmplitude * Math.sin(sineFrequency * (time - initialTime)));
			actualICPPositionYoFramePoint.set(actualICPPosition);

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
					updatePointsVis(footstepList);
				}
			}

			if (testFootslipCompensation)
			{
				transferToFootstep.setX(transferToFootstep.getX() + 1e-5);
				transferToFootstep.setY(transferToFootstep.getY() + 2e-5);

				if (visualize)
				{
					updatePointsVis(footstepList);
				}
			}

			initialICPPosition.set(icpPositionToPack);

			time += deltaT;
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

	private void updateFootstepsFromPush(FrameTupleArrayList<FramePoint> footstepList)
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

			plotFootPolygon(footstepList, listOfFootPolygons);
		}
	}

	public void plotFootPolygon(FrameTupleArrayList<FramePoint> footstepList, ArrayList<ArrayList<YoFrameLineSegment2d>> listOfFootLineSegments)
	{
		FramePoint2d toeLeft = new FramePoint2d(ReferenceFrame.getWorldFrame());
		FramePoint2d toeRight = new FramePoint2d(ReferenceFrame.getWorldFrame());
		FramePoint2d heelRight = new FramePoint2d(ReferenceFrame.getWorldFrame());
		FramePoint2d heelLeft = new FramePoint2d(ReferenceFrame.getWorldFrame());

		int iters = Math.min(footstepList.size(), listOfFootLineSegments.size());
		for (int i = 0; i < iters; i++)
		{
			FramePoint foot = footstepList.get(i);
			toeLeft.set(foot.getX() + footLength / 2, foot.getY() - footWidth / 2);
			toeRight.set(foot.getX() + footLength / 2, foot.getY() + footWidth / 2);
			heelLeft.set(foot.getX() - footLength / 2, foot.getY() - footWidth / 2);
			heelRight.set(foot.getX() - footLength / 2, foot.getY() + footWidth / 2);

			ArrayList<YoFrameLineSegment2d> footLineSegments = listOfFootLineSegments.get(i);
			footLineSegments.get(0).set(toeLeft, toeRight);
			footLineSegments.get(1).set(toeRight, heelRight);
			footLineSegments.get(2).set(heelRight, heelLeft);
			footLineSegments.get(3).set(heelLeft, toeLeft);
		}

	}
	
	private void updatePointsVis(FrameTupleArrayList<FramePoint> footLocations)
   {
      for(int i = 0; i < Math.min(footstepYoFramePoints.size(),footLocations.size()); i++)
      {
         footstepYoFramePoints.get(i).set(footLocations.get(i));
      }
      
      plotFootPolygon(footLocations, listOfFootPolygons);
      
      for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
      {
         constantCoPsViz.get(i).set(icpPlanner.getConstantCentroidalMomentumPivots().get(i).getFramePointCopy());
      }

      for (int i = 0; i < maxNumberOfConsideredFootsteps - 1; i++)
      {
         icpFootCenterCornerPointsViz.get(i).set(icpPlanner.getCapturePointCornerPoints().get(i).getFramePointCopy());
      }
   }
}
