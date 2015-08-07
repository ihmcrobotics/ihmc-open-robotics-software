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
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFrameLineSegment2d;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;

public class NewInstantaneousCapturePointPlannerTest
{
	private YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

	private boolean visualize = false;
	private boolean testPush = false;
	boolean cancelPlan = true;
	private final Random random = new Random();

	private PointAndLinePlotter pointAndLinePlotter = new PointAndLinePlotter(registry);
	private YoGraphicsListRegistry yoGraphicsListRegistry = null;
	private SimulationConstructionSet scs = null;
	private DoubleYoVariable timeYoVariable = null;
	private DoubleYoVariable stopSignalTime = new DoubleYoVariable("stopSignalTime", registry);

	YoFramePoint tmpPreviousICPPosition = new YoFramePoint("PreviousICPPosition", ReferenceFrame.getWorldFrame(), registry);

	private YoFramePoint icpArrowTip = null;
	private YoFramePoint icpPositionYoFramePoint = null;
	private YoFramePoint singleSupportEndICPPosition = null;
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
			return 0.2;
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
	private int numberOfStepsInStepList = 4;
	private int maxNumberOfConsideredFootsteps = testICPPlannerParams.getNumberOfFootstepsToConsider();
	private NewInstantaneousCapturePointPlanner icpPlanner;

	private YoFrameLineSegment2d icpVelocityLineSegment = null;

	private double scsPlaybackRate = 0.25;

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

		footstepYoFramePoints = null;

		icpFootCenterCornerPointsViz = null;

		constantCoPsViz = null;

		doubleSupportStartICPYoFramePoint = null;
		doubleSupportEndICPYoFramePoint = null;

		icpVelocityLineSegment = null;
	}

	@EstimatedDuration(duration = 2.9)
	@Test(timeout = 300000)
	public void visualizePlanner()
	{
		icpPlanner = new NewInstantaneousCapturePointPlanner(maxNumberOfConsideredFootsteps, testICPPlannerParams, registry,
				yoGraphicsListRegistry);

		stopSignalTime.set(1.9e100);

		createVisualizers(maxNumberOfConsideredFootsteps);

		RobotSide stepSide = RobotSide.LEFT;
		double stepLength = 0.3;
		double halfStepWidth = 0.1;
		boolean startSquaredUp = true;

		double comHeight = 1.0;
		double gravitationalAcceleration = 9.81;

		double omega0 = Math.sqrt(gravitationalAcceleration / comHeight);

		ArrayList<FramePoint> footLocations = createABunchOfUniformWalkingSteps(stepSide, startSquaredUp, numberOfStepsInStepList,
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
					initialTime, omega0, initialICPPosition, footLocations);

			singleSupportEndICPPosition.set(icpPosition);

			initialTime = initialTime + singleSupportDuration;

			icpPlanner.initializeDoubleSupport(icpPosition, icpVelocity, initialTime, footLocations);
			icpPlanner.packDesiredCapturePointPositionVelocityAndAcceleration(initialICPPosition, initialICPVelocity,
					initialICPAcceleration, initialTime);

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

	private ArrayList<FramePoint> createABunchOfUniformWalkingSteps(RobotSide stepSide, boolean startSquaredUp,
			int numberOfStepsInStepList, double stepLength, double halfStepWidth)
	{
		ArrayList<FramePoint> footLocations = new ArrayList<FramePoint>();

		double height = 0.5;

		if (startSquaredUp)
		{
			FramePoint firstStepLocation = new FramePoint(ReferenceFrame.getWorldFrame());
			firstStepLocation.set(0, stepSide.negateIfRightSide(halfStepWidth), height);
			footLocations.add(firstStepLocation);

			stepSide = stepSide.getOppositeSide();
		}

		for (int i = 0; i < numberOfStepsInStepList; i++)
		{
			FramePoint stepLocation = new FramePoint(ReferenceFrame.getWorldFrame());
			stepLocation.set(i * stepLength, stepSide.negateIfRightSide(halfStepWidth), height);

			footLocations.add(stepLocation);
			stepSide = stepSide.getOppositeSide();
		}

		return footLocations;
	}

	private void createVisualizers(int maxNumberOfConsideredFootsteps)
	{
		ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

		Robot robot = new Robot("TestRobot");
		
		SimulationConstructionSetParameters simulationConstructionSetParameters = SimulationConstructionSetParameters.createFromEnvironmentVariables();
		scs = new SimulationConstructionSet(robot, simulationConstructionSetParameters);

		scs.setDT(deltaT, 1);
		scs.changeBufferSize(16000);
		scs.setPlaybackRealTimeRate(scsPlaybackRate);
		scs.setPlaybackDesiredFrameRate(scsPlaybackDesiredFrameRate);

		robot.getRobotsYoVariableRegistry().addChild(registry);
		timeYoVariable = robot.getYoTime();

		pointAndLinePlotter.createAndShowOverheadPlotterInSCS(scs);

		icpArrowTip = new YoFramePoint("icpVelocityTip", "", worldFrame, registry);
		singleSupportEndICPPosition = new YoFramePoint("flegm", "", worldFrame, registry);
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
		pointAndLinePlotter.plotYoFramePoint("icpFlegmPosition", singleSupportEndICPPosition, YoAppearance.Crimson(), 0.004);
		pointAndLinePlotter.plotYoFramePoint("cmpPosition", cmpPositionYoFramePoint, YoAppearance.Magenta(), 0.005);
		pointAndLinePlotter.plotLineSegment("icpVelocity", icpVelocityLineSegment, Color.gray);

		pointAndLinePlotter.plotYoFramePoint("doubleSupportICPStart", doubleSupportStartICPYoFramePoint, YoAppearance.Cyan(), 0.003);
		pointAndLinePlotter.plotYoFramePoint("doubleSupportICPEnd", doubleSupportEndICPYoFramePoint, YoAppearance.Cyan(), 0.004);

		yoGraphicsListRegistry = pointAndLinePlotter.getDynamicGraphicObjectsListRegistry();
	}

	private void simulateForwardAndCheckSingleSupport(FramePoint icpPositionToPack, FrameVector icpVelocityToPack,
			FrameVector icpAccelerationToPack, FramePoint ecmpPositionToPack, NewInstantaneousCapturePointPlanner icpPlanner,
			double singleSupportDuration, double initialTime, double omega0, FramePoint initialICPPosition,
			ArrayList<FramePoint> footstepList)
	{
		for (double time = initialTime + deltaT; time <= initialTime + singleSupportDuration; time = time + deltaT)
		{
			icpPlanner.packDesiredCapturePointPositionVelocityAndAcceleration(icpPositionToPack, icpVelocityToPack, icpAccelerationToPack,
					time);

			if (visualize)
			{
				visualizeICPAndECMP(icpPositionToPack, icpVelocityToPack, icpPlanner.getConstantCentroidalMomentumPivots().get(0)
						.getFramePointCopy(), time);
			}

			initialICPPosition.set(icpPositionToPack);

			if (testPush)
			{
				if (time > initialTime + 0.2)
				{
					updateFootstepsFromPush(footstepList);
					icpPlanner.updateForSingleSupportPush(footstepList, time);
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
					testPush = false;
				}
			}
		}
	}

	private void simulateForwardAndCheckDoubleSupport(ArrayList<FramePoint> footstepList, FramePoint icpPositionToPack,
			FrameVector icpVelocityToPack, FrameVector icpAccelerationToPack, FramePoint ecmpPositionToPack,
			NewInstantaneousCapturePointPlanner icpPlanner, double doubleSupportDuration, double initialTime, double deltaT, double omega0,
			FramePoint initialICPPosition, boolean cancelPlan)
	{
		for (double time = initialTime + deltaT; time <= initialTime + doubleSupportDuration; time = time + deltaT)
		{
			icpPlanner.packDesiredCapturePointPositionVelocityAndAcceleration(icpPositionToPack, icpVelocityToPack, icpAccelerationToPack,
					time);

			if (visualize)
			{
				visualizeICPAndECMP(icpPositionToPack, icpVelocityToPack, icpPlanner.getConstantCentroidalMomentumPivots().get(0)
						.getFramePointCopy(), time);
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

	private void visualizeICPAndECMP(FramePoint icpPosition, FrameVector icpVelocity, FramePoint ecmpPosition, double time)
	{
		icpPositionYoFramePoint.set(icpPosition);
		icpVelocityYoFramePoint.set(icpVelocity);

		PointAndLinePlotter.setEndPointGivenStartAndAdditionalVector(icpArrowTip, icpPosition.getPointCopy(), icpVelocity.getVectorCopy(),
				0.5);
		Point2d icpPosition2d = new Point2d(icpPosition.getX(), icpPosition.getY());
		PointAndLinePlotter.setLineSegmentBasedOnStartAndEndFramePoints(icpVelocityLineSegment, icpPosition2d, icpArrowTip
				.getFramePoint2dCopy().getPointCopy());

		// cmpPositionYoFramePoint.set(ecmpPosition);

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

		for (int i = 0; i < Math.min(footstepList.size(), footstepYoFramePoints.size()); i++)
		{
			footstepYoFramePoints.get(i).set(footstepList.get(i));
		}
	}
}
