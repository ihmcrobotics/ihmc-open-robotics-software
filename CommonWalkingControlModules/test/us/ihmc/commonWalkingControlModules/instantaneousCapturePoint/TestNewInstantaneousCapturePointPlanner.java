package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;

import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.DoubleSupportFootCenterToToeICPComputer;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.PointAndLinePlotter;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFrameLineSegment2d;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public class TestNewInstantaneousCapturePointPlanner
{
	private YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

	private boolean visualize = true;
	private boolean testPush = true;
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
		public int getNumberOfFootstepsToStop()
		{
			return 2;
		}

		@Override
		public double getIsDoneTimeThreshold()
		{
			return -1e-4;
		}
	};

	private double singleSupportDuration = testICPPlannerParams.getSingleSupportDuration();
	private double doubleSupportDuration = testICPPlannerParams.getDoubleSupportDuration();
	private double doubleSupportInitialTransferDuration = testICPPlannerParams.getDoubleSupportInitialTransferDuration();
	private int numberOfStepsInStepList = 3;
	private int maxNumberOfConsideredFootsteps = testICPPlannerParams.getNumberOfFootstepsToConsider();
	private NewInstantaneousCapturePointPlanner icpPlanner;

	private YoFrameLineSegment2d icpVelocityLineSegment = null;

	private double scsPlaybackRate = 1;

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

	@Test
	public void test1()
	{
		icpPlanner = new NewInstantaneousCapturePointPlanner(maxNumberOfConsideredFootsteps, testICPPlannerParams,registry, yoGraphicsListRegistry);

		stopSignalTime.set(1.9e100);

		createVisualizers(maxNumberOfConsideredFootsteps);

		RobotSide stepSide = RobotSide.LEFT;
		double stepLength = 0.3;
		double halfStepWidth = 0.1;
		boolean startSquaredUp = true;

		double comHeight = 1.0;
		double gravitationalAcceleration = 9.81;

		double omega0 = Math.sqrt(gravitationalAcceleration / comHeight);

		ArrayList<YoFramePoint> footLocations= createABunchOfUniformWalkingSteps(stepSide, startSquaredUp, numberOfStepsInStepList, stepLength,
				halfStepWidth);

		YoFramePoint initialICPPosition = new YoFramePoint("initialICP", ReferenceFrame.getWorldFrame(), registry);
		YoFrameVector initialICPVelocity = new YoFrameVector("initialICPVelocity", ReferenceFrame.getWorldFrame(), registry);
		YoFrameVector initialICPAcceleration = new YoFrameVector("initialICPAcceleration", ReferenceFrame.getWorldFrame(), registry);

		YoFramePoint icpPosition = new YoFramePoint("icpPosition", ReferenceFrame.getWorldFrame(), registry);
		YoFrameVector icpVelocity = new YoFrameVector("icpVelocity", ReferenceFrame.getWorldFrame(), registry);
		YoFrameVector icpAcceleration = new YoFrameVector("icpAcceleration", ReferenceFrame.getWorldFrame(), registry);

		YoFramePoint cmpPosition = new YoFramePoint("cmpPosition", ReferenceFrame.getWorldFrame(), registry);

		double initialTime = 0.0;

		initialICPPosition.set(footLocations.get(0));
		initialICPPosition.add(footLocations.get(1));
		initialICPPosition.scale(0.5);

		icpPlanner.initializeDoubleSupport(initialICPPosition, initialICPVelocity, omega0,
				initialTime, footLocations);

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
				constantCoPsViz.get(i).set(icpPlanner.getConstantCentersOfPressure().get(i));
			}

			for (int i = 0; i < maxNumberOfConsideredFootsteps - 1; i++)
			{
				icpFootCenterCornerPointsViz.get(i).set(icpPlanner.getCapturePointCornerPoints().get(i));
			}
		}

		simulateForwardAndCheckDoubleSupport(icpPosition, icpVelocity, icpAcceleration, cmpPosition, icpPlanner, doubleSupportInitialTransferDuration,
				initialTime, deltaT, omega0, initialICPPosition);

		initialTime = initialTime + doubleSupportInitialTransferDuration;

		footLocations.remove(0);

		for (int i = 0; i < Math.min(footLocations.size(), footstepYoFramePoints.size()); i++)
		{
			footstepYoFramePoints.get(i).set(footLocations.get(i));
		}

		while (footLocations.size() >= 2)
		{

			icpPlanner.initializeSingleSupport(omega0, initialTime, footLocations);

			if (visualize)
			{
				for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
				{
					constantCoPsViz.get(i).set(icpPlanner.getConstantCentersOfPressure().get(i).getFramePointCopy());
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

			icpPlanner.initializeDoubleSupport(icpPosition, icpVelocity, omega0, initialTime,
					footLocations);
			icpPlanner.packDesiredCapturePointPositionVelocityAndAcceleration(initialICPPosition, initialICPVelocity,
					initialICPAcceleration, initialTime);

			if (visualize)
			{
				for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
				{
					constantCoPsViz.get(i).set(icpPlanner.getConstantCentersOfPressure().get(i).getFramePointCopy());
				}

				for (int i = 0; i < maxNumberOfConsideredFootsteps - 1; i++)
				{
					icpFootCenterCornerPointsViz.get(i).set(icpPlanner.getCapturePointCornerPoints().get(i).getFramePointCopy());
				}
			}

			simulateForwardAndCheckDoubleSupport(icpPosition, icpVelocity, icpAcceleration, cmpPosition, icpPlanner, doubleSupportDuration,
					initialTime, deltaT, omega0, initialICPPosition);

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

	private ArrayList<YoFramePoint> createABunchOfUniformWalkingSteps(RobotSide stepSide, boolean startSquaredUp,
			int numberOfStepsInStepList, double stepLength, double halfStepWidth)
	{
		ArrayList<YoFramePoint> footLocations = new ArrayList<YoFramePoint>();

		double height = 0.5;

		if (startSquaredUp)
		{
			YoFramePoint firstStepLocation = new YoFramePoint("stepListElement" + 100, "", ReferenceFrame.getWorldFrame(), registry);
			firstStepLocation.set(0, stepSide.negateIfRightSide(halfStepWidth), height);
			footLocations.add(firstStepLocation);

			stepSide = stepSide.getOppositeSide();
		}

		for (int i = 0; i < numberOfStepsInStepList; i++)
		{
			YoFramePoint stepLocation = new YoFramePoint("stepListElement" + i, "", ReferenceFrame.getWorldFrame(), registry);
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
		scs = new SimulationConstructionSet(robot);

		scs.setDT(deltaT, 1);
		scs.changeBufferSize(16000);
		scs.setPlaybackRealTimeRate(scsPlaybackRate);
		scs.setPlaybackDesiredFrameRate(scsPlaybackDesiredFrameRate);

		robot.getRobotsYoVariableRegistry().addChild(registry);
		timeYoVariable = robot.getYoTime();

		pointAndLinePlotter.createAndShowOverheadPlotterInSCS(scs);

		icpArrowTip = new YoFramePoint("icpVelocityTip", "", worldFrame, registry);
		singleSupportEndICPPosition = new YoFramePoint("flegm","",worldFrame,registry);
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

	private void simulateForwardAndCheckSingleSupport(YoFramePoint icpPositionToPack, YoFrameVector icpVelocityToPack,
			YoFrameVector icpAccelerationToPack, YoFramePoint ecmpPositionToPack, NewInstantaneousCapturePointPlanner icpPlanner,
			double singleSupportDuration, double initialTime, double omega0, YoFramePoint initialICPPosition,
			ArrayList<YoFramePoint> footstepList)
	{
		for (double time = initialTime + deltaT; time <= initialTime + singleSupportDuration; time = time + deltaT)
		{
			icpPlanner.packDesiredCapturePointPositionVelocityAndAcceleration(icpPositionToPack, icpVelocityToPack, icpAccelerationToPack,
					time);

			if (visualize)
			{
				visualizeICPAndECMP(icpPlanner.getDesiredCapturePointPosition(), icpPlanner.getDesiredCapturePointVelocity(), icpPlanner
						.getConstantCentersOfPressure().get(0), time);
			}

			initialICPPosition.set(icpPositionToPack);
			
			if(testPush)
			{
				if(time > initialTime + 0.2)
				{
					updateFootstepsFromPush(footstepList);
					icpPlanner.updateForSingleSupportPush(footstepList, time);
					if (visualize)
					{
						for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
						{
							constantCoPsViz.get(i).set(icpPlanner.getConstantCentersOfPressure().get(i));
						}

						for (int i = 0; i < maxNumberOfConsideredFootsteps - 1; i++)
						{
							icpFootCenterCornerPointsViz.get(i).set(icpPlanner.getCapturePointCornerPoints().get(i));
						}
					}
					testPush=false;
				}
			}
		}
	}

	private void simulateForwardAndCheckDoubleSupport(YoFramePoint icpPositionToPack, YoFrameVector icpVelocityToPack,
			YoFrameVector icpAccelerationToPack, YoFramePoint ecmpPositionToPack, NewInstantaneousCapturePointPlanner icpPlanner,
			double doubleSupportDuration, double initialTime, double deltaT, double omega0, YoFramePoint initialICPPosition)
	{
		for (double time = initialTime + deltaT; time <= initialTime + doubleSupportDuration; time = time + deltaT)
		{
			icpPlanner.packDesiredCapturePointPositionVelocityAndAcceleration(icpPositionToPack, icpVelocityToPack, icpAccelerationToPack,
					time);

			if (visualize)
			{
				visualizeICPAndECMP(icpPlanner.getDesiredCapturePointPosition(), icpPlanner.getDesiredCapturePointVelocity(), icpPlanner
						.getConstantCentersOfPressure().get(0), time);
			}

			initialICPPosition.set(icpPositionToPack);
		}
	}

	private void visualizeICPAndECMP(YoFramePoint icpPosition, YoFrameVector icpVelocity, YoFramePoint ecmpPosition, double time)
	{
		icpPositionYoFramePoint.set(icpPosition);
		icpVelocityYoFramePoint.set(icpVelocity);

		PointAndLinePlotter.setEndPointGivenStartAndAdditionalVector(icpArrowTip, icpPosition.getPoint3dCopy(),
				icpVelocity.getVector3dCopy(), 0.5);
		Point2d icpPosition2d = new Point2d(icpPosition.getX(), icpPosition.getY());
		PointAndLinePlotter.setLineSegmentBasedOnStartAndEndFramePoints(icpVelocityLineSegment, icpPosition2d, icpArrowTip
				.getFramePoint2dCopy().getPointCopy());

		// cmpPositionYoFramePoint.set(ecmpPosition);

		timeYoVariable.set(time);
		scs.tickAndUpdate();
	}
	
	private void updateFootstepsFromPush(ArrayList<YoFramePoint> footstepList)
	{	
		double tmpx = random.nextDouble()*0.1;
		double tmpy = random.nextDouble()*0.05;
		
		for(int i = 1; i < footstepList.size()-1; i++)
		{	
			footstepList.get(i).setX(footstepList.get(i).getX()+tmpx);
			footstepList.get(i).setY(footstepList.get(i).getY()+tmpy);
		}
		
		for (int i = 0; i < Math.min(footstepList.size(), footstepYoFramePoints.size()); i++)
		{
			footstepYoFramePoints.get(i).set(footstepList.get(i));
		}
	}
}
