package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import static org.junit.Assert.fail;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.tools.random.RandomTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFrameLineSegment2d;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;

@DeployableTestClass(targets = {TestPlanTarget.UI})
public class SmoothICPComputerTest
{
   private static final boolean USE_ASSERTS = true; //false;

   private boolean visualize = false;

   private PointAndLinePlotter pointAndLinePlotter = null;
   private YoGraphicsListRegistry yoGraphicsListRegistry = null;
   private SimulationConstructionSet scs = null;
   private DoubleYoVariable timeYoVariable = null;


// private YoVariableRegistry registry = null;
   private YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private YoFramePoint icpArrowTip = null;
   private YoFramePoint icpPositionYoFramePoint = null;
   private YoFramePoint icpVelocityYoFramePoint = null;
   private YoFramePoint ecmpPositionYoFramePoint = null;

   private ArrayList<YoFramePoint> footstepYoFramePoints = null;

   private YoFramePoint doubleSupportStartICPYoFramePoint = null;
   private YoFramePoint doubleSupportEndICPYoFramePoint = null;




   private YoFrameLineSegment2d icpVelocityLineSegment = null;

   @Before
   public void createRegistryBeforeTests()
   {
      registry = new YoVariableRegistry(getClass().getSimpleName());
   }

   private final DoubleYoVariable stopSignalTime = new DoubleYoVariable("stopSignalTime", registry);
   private final BooleanYoVariable stopSignalFlag = new BooleanYoVariable("stopSignalFlag", registry);

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

      doubleSupportStartICPYoFramePoint = null;
      doubleSupportEndICPYoFramePoint = null;

      ecmpPositionYoFramePoint = null;
      icpVelocityLineSegment = null;
   }

	@DeployableTestMethod(estimatedDuration = 0.8)
	@Test(timeout = 30000)
   public void testTypicalFourStepExample()
   {
      
      stopSignalTime.set(1.9e100);
      
      double deltaT = 0.001;

      int maxNumberOfConsideredFootsteps = 4;

      createVisualizers(maxNumberOfConsideredFootsteps, visualize);

      double doubleSupportFirstStepFraction = 0.5;

      SmoothICPComputer smoothICPComputer = new SmoothICPComputer(doubleSupportFirstStepFraction, maxNumberOfConsideredFootsteps, registry,
                                               yoGraphicsListRegistry);

      RobotSide stepSide = RobotSide.LEFT;
      int numberOfStepsInStepList = 7;
      double stepLength = 0.3;
      double halfStepWidth = 0.1;
      boolean startSquaredUp = true;

      double singleSupportDuration = 0.6;
      double doubleSupportDuration = 0.2;
      double doubleSupportInitialTransferDuration = 0.4;

      double eCMPHeight = 1.0;
      double gravitationalAcceleration = 9.81;

      double omega0 = Math.sqrt(gravitationalAcceleration / eCMPHeight);

      ArrayList<YoFramePoint> stepList = createABunchOfUniformWalkingSteps(stepSide, startSquaredUp, numberOfStepsInStepList, stepLength, halfStepWidth);

//    ArrayList<YoFramePoint> stepList = createABunchOfRandomWalkingSteps(stepSide, startSquaredUp, numberOfStepsInStepList, stepLength, halfStepWidth);

      Point3d initialICPPosition = new Point3d();
      Point3d initialECMPPosition = new Point3d();
      Vector3d initialICPVelocity = new Vector3d();

      Point3d icpPosition = new Point3d();
      Vector3d icpVelocity = new Vector3d();
      Point3d ecmpPosition = new Point3d();

      double initialTime = 1.1;
      ArrayList<FramePoint> footLocations = getFirstSteps(stepList, 4);
      if (visualize)
      {
         for (int i = 0; i < Math.min(footLocations.size(), footstepYoFramePoints.size()); i++)
         {
            footstepYoFramePoints.get(i).set(footLocations.get(i));
         }
      }

      initialICPPosition.set(footLocations.get(0).getPoint());
      initialICPPosition.add(footLocations.get(1).getPoint());
      initialICPPosition.scale(0.5);

      boolean stopIfReachedEnd = false;
      smoothICPComputer.initializeDoubleSupportInitialTransfer(footLocations, initialICPPosition, singleSupportDuration, doubleSupportDuration,
              doubleSupportInitialTransferDuration, omega0, initialTime, stopIfReachedEnd);
      smoothICPComputer.getICPPositionAndVelocity(initialICPPosition, initialICPVelocity, initialECMPPosition, initialTime);

      if (visualize)
      {
         doubleSupportStartICPYoFramePoint.set(smoothICPComputer.getDoubleSupportStartICP());
         doubleSupportEndICPYoFramePoint.set(smoothICPComputer.getDoubleSupportEndICP());
      }

      Point3d transferFromFoot = footLocations.get(0).getPoint();
      simulateForwardAndCheckDoubleSupport(icpPosition, icpVelocity, ecmpPosition, smoothICPComputer, doubleSupportInitialTransferDuration, initialTime,
              deltaT, omega0, initialICPPosition, stepList, footLocations);

      initialTime = initialTime + doubleSupportInitialTransferDuration;
      stepList.remove(0);

      if (visualize)
      {
         doubleSupportStartICPYoFramePoint.set(smoothICPComputer.getDoubleSupportStartICP());
         doubleSupportEndICPYoFramePoint.set(smoothICPComputer.getDoubleSupportEndICP());
      }

      while (stepList.size() >= 2)
      {
         footLocations = getFirstSteps(stepList, 4);

         if (visualize)
         {
            for (int i = 0; i < Math.min(footLocations.size(), footstepYoFramePoints.size()); i++)
            {
               footstepYoFramePoints.get(i).set(footLocations.get(i));
            }
         }

         stopIfReachedEnd = (stepList.size() <= maxNumberOfConsideredFootsteps);
         smoothICPComputer.initializeSingleSupport(footLocations, singleSupportDuration, doubleSupportDuration, omega0, initialTime, stopIfReachedEnd);

         if (visualize)
         {
            doubleSupportStartICPYoFramePoint.set(smoothICPComputer.getDoubleSupportStartICP());
            doubleSupportEndICPYoFramePoint.set(smoothICPComputer.getDoubleSupportEndICP());
         }

         smoothICPComputer.getICPPositionAndVelocity(initialICPPosition, initialICPVelocity, initialECMPPosition, initialTime);


         if (USE_ASSERTS)
         {
            JUnitTools.assertTuple3dEquals(icpPosition, initialICPPosition, 3e-3);
            JUnitTools.assertTuple3dEquals(icpVelocity, initialICPVelocity, 1e-2);
            JUnitTools.assertTuple3dEquals(ecmpPosition, initialECMPPosition, 3e-3);
         }

         transferFromFoot = footLocations.get(0).getPoint();


         simulateForwardAndCheckSingleSupport(icpPosition, icpVelocity, ecmpPosition, smoothICPComputer, singleSupportDuration, initialTime, omega0,
                 initialICPPosition, transferFromFoot, stepList, footLocations);

         initialTime = initialTime + singleSupportDuration;
         smoothICPComputer.initializeDoubleSupport(footLocations, singleSupportDuration, doubleSupportDuration, omega0, initialTime, stopIfReachedEnd);
         smoothICPComputer.getICPPositionAndVelocity(initialICPPosition, initialICPVelocity, initialECMPPosition, initialTime);

         if (USE_ASSERTS)
         {
            JUnitTools.assertTuple3dEquals(icpPosition, initialICPPosition, 1e-3);
            JUnitTools.assertTuple3dEquals(icpVelocity, initialICPVelocity, 1e-2);
            JUnitTools.assertTuple3dEquals(ecmpPosition, initialECMPPosition, 3e-3);
         }

         if (visualize)
         {
            doubleSupportStartICPYoFramePoint.set(smoothICPComputer.getDoubleSupportStartICP());
            doubleSupportEndICPYoFramePoint.set(smoothICPComputer.getDoubleSupportEndICP());
         }

         simulateForwardAndCheckDoubleSupport(icpPosition, icpVelocity, ecmpPosition, smoothICPComputer, doubleSupportDuration, initialTime, deltaT, omega0,
                 initialICPPosition, stepList, footLocations);

         initialTime = initialTime + doubleSupportDuration;

         stepList.remove(0);
      }

      double timeToSimulateAtEnd = 3.0;
      initialICPPosition.set(icpPosition);
      simulateForwardAndCheckDoubleSupport(icpPosition, icpVelocity, ecmpPosition, smoothICPComputer, timeToSimulateAtEnd, initialTime, deltaT, omega0,
              initialICPPosition, stepList, footLocations);

      if (visualize)
      {
         pointAndLinePlotter.addPointsAndLinesToSCS(scs);

         scs.startOnAThread();
         ThreadTools.sleepForever();
      }
   }

	@DeployableTestMethod(estimatedDuration = 0.7)
	@Test(timeout = 30000)
   public void testTypicalFourStepExampleWithSuddenStop()
   {
      double deltaT = 0.001;
      stopSignalTime.set(1.9e100);



      int maxNumberOfConsideredFootsteps = 4;

      createVisualizers(maxNumberOfConsideredFootsteps, visualize);

      double doubleSupportFirstStepFraction = 0.5;

      SmoothICPComputer smoothICPComputer = new SmoothICPComputer(doubleSupportFirstStepFraction, maxNumberOfConsideredFootsteps, registry,
                                               yoGraphicsListRegistry);

      RobotSide stepSide = RobotSide.LEFT;
      int numberOfStepsInStepList = 7;
      double stepLength = 0.3;
      double halfStepWidth = 0.1;
      boolean startSquaredUp = true;

      double singleSupportDuration = 0.6;
      double doubleSupportDuration = 0.2;
      double doubleSupportInitialTransferDuration = 0.4;

      double eCMPHeight = 1.0;
      double gravitationalAcceleration = 9.81;

      double omega0 = Math.sqrt(gravitationalAcceleration / eCMPHeight);

      ArrayList<YoFramePoint> stepList = createABunchOfUniformWalkingSteps(stepSide, startSquaredUp, numberOfStepsInStepList, stepLength, halfStepWidth);

//    ArrayList<YoFramePoint> stepList = createABunchOfRandomWalkingSteps(stepSide, startSquaredUp, numberOfStepsInStepList, stepLength, halfStepWidth);

      Point3d initialICPPosition = new Point3d();
      Point3d initialECMPPosition = new Point3d();
      Vector3d initialICPVelocity = new Vector3d();

      Point3d icpPosition = new Point3d();
      Vector3d icpVelocity = new Vector3d();
      Point3d ecmpPosition = new Point3d();

      double initialTime = 0.0;
      ArrayList<FramePoint> footLocations = getFirstSteps(stepList, 4);
      if (visualize)
      {
         for (int i = 0; i < Math.min(footLocations.size(), footstepYoFramePoints.size()); i++)
         {
            footstepYoFramePoints.get(i).set(footLocations.get(i));
         }
      }

      initialICPPosition.set(footLocations.get(0).getPoint());
      initialICPPosition.add(footLocations.get(1).getPoint());
      initialICPPosition.scale(0.5);

      boolean stopIfReachedEnd = false;
      smoothICPComputer.initializeDoubleSupportInitialTransfer(footLocations, initialICPPosition, singleSupportDuration, doubleSupportDuration,
              doubleSupportInitialTransferDuration, omega0, initialTime, stopIfReachedEnd);
      smoothICPComputer.getICPPositionAndVelocity(initialICPPosition, initialICPVelocity, initialECMPPosition, initialTime);

      if (visualize)
      {
         doubleSupportStartICPYoFramePoint.set(smoothICPComputer.getDoubleSupportStartICP());
         doubleSupportEndICPYoFramePoint.set(smoothICPComputer.getDoubleSupportEndICP());
      }

      Point3d transferFromFoot = footLocations.get(0).getPoint();
      simulateForwardAndCheckDoubleSupport(icpPosition, icpVelocity, ecmpPosition, smoothICPComputer, doubleSupportInitialTransferDuration, initialTime,
              deltaT, omega0, initialICPPosition, stepList, footLocations);

      initialTime = initialTime + doubleSupportInitialTransferDuration;

      stepList.remove(0);

      stopSignalFlag.set(timeYoVariable.getDoubleValue() >= stopSignalTime.getDoubleValue());

      if (stopSignalFlag.getBooleanValue())
      {
         removeAllStepsFromStepListExceptFirstTwo(stepList);
         footLocations.clear();
         footLocations.addAll(getFirstSteps(stepList, 4));
         
         for (int i = 1; i < footstepYoFramePoints.size(); i++)
         {
            footstepYoFramePoints.get(i).set(footLocations.get(Math.min(footLocations.size(), footstepYoFramePoints.size())-1));
         }
      }

      if (visualize)
      {
         doubleSupportStartICPYoFramePoint.set(smoothICPComputer.getDoubleSupportStartICP());
         doubleSupportEndICPYoFramePoint.set(smoothICPComputer.getDoubleSupportEndICP());
      }

      while (stepList.size() >= 2)
      {
         footLocations = getFirstSteps(stepList, 4);

         if (visualize)
         {
            for (int i = 0; i < Math.min(footLocations.size(), footstepYoFramePoints.size()); i++)
            {
               footstepYoFramePoints.get(i).set(footLocations.get(i));
            }
         }

         stopIfReachedEnd = (stepList.size() <= maxNumberOfConsideredFootsteps);
         smoothICPComputer.initializeSingleSupport(footLocations, singleSupportDuration, doubleSupportDuration, omega0, initialTime, stopIfReachedEnd);

         if (visualize)
         {
            doubleSupportStartICPYoFramePoint.set(smoothICPComputer.getDoubleSupportStartICP());
            doubleSupportEndICPYoFramePoint.set(smoothICPComputer.getDoubleSupportEndICP());
         }

         smoothICPComputer.getICPPositionAndVelocity(initialICPPosition, initialICPVelocity, initialECMPPosition, initialTime);


         if (USE_ASSERTS)
         {
            JUnitTools.assertTuple3dEquals(icpPosition, initialICPPosition, 3e-3);
            JUnitTools.assertTuple3dEquals(icpVelocity, initialICPVelocity, 1e-2);
            JUnitTools.assertTuple3dEquals(ecmpPosition, initialECMPPosition, 3e-3);
         }

         transferFromFoot = footLocations.get(0).getPoint();

         simulateForwardAndCheckSingleSupport(icpPosition, icpVelocity, ecmpPosition, smoothICPComputer, singleSupportDuration, initialTime, omega0,
                 initialICPPosition, transferFromFoot, stepList, footLocations);

         initialTime = initialTime + singleSupportDuration;

         smoothICPComputer.initializeDoubleSupport(footLocations, singleSupportDuration, doubleSupportDuration, omega0, initialTime, stopIfReachedEnd);
         smoothICPComputer.getICPPositionAndVelocity(initialICPPosition, initialICPVelocity, initialECMPPosition, initialTime);

         if (USE_ASSERTS)
         {
            JUnitTools.assertTuple3dEquals(icpPosition, initialICPPosition, 1e-3);
            JUnitTools.assertTuple3dEquals(icpVelocity, initialICPVelocity, 1e-2);
            JUnitTools.assertTuple3dEquals(ecmpPosition, initialECMPPosition, 3e-3);
         }

         if (visualize)
         {
            doubleSupportStartICPYoFramePoint.set(smoothICPComputer.getDoubleSupportStartICP());
            doubleSupportEndICPYoFramePoint.set(smoothICPComputer.getDoubleSupportEndICP());
         }

         simulateForwardAndCheckDoubleSupport(icpPosition, icpVelocity, ecmpPosition, smoothICPComputer, doubleSupportDuration, initialTime, deltaT, omega0,
                 initialICPPosition, stepList, footLocations);

         initialTime = initialTime + doubleSupportDuration;


         stepList.remove(0);
      }

      double timeToSimulateAtEnd = 3.0;
      initialICPPosition.set(icpPosition);
      simulateForwardAndCheckDoubleSupport(icpPosition, icpVelocity, ecmpPosition, smoothICPComputer, timeToSimulateAtEnd, initialTime, deltaT, omega0,
              initialICPPosition, stepList, footLocations);

      if (visualize)
      {
         pointAndLinePlotter.addPointsAndLinesToSCS(scs);

         scs.startOnAThread();
         ThreadTools.sleepForever();
      }
   }



   private void removeAllStepsFromStepListExceptFirstTwo(ArrayList<YoFramePoint> stepList)
   {
      int stepListSize = stepList.size();

//    if (stepListSize > 2)
//    {
      for (int i = stepListSize - 1; i >= 2; i--)
      {
         stepList.remove(i);
      }

//    }  
   }

   private ArrayList<FramePoint> getFirstSteps(ArrayList<YoFramePoint> stepList, int maximumNumberOfStepsToReturn)
   {
      ArrayList<FramePoint> ret = new ArrayList<FramePoint>();

      int numberOfStepsToReturn = Math.min(stepList.size(), maximumNumberOfStepsToReturn);
      for (int i = 0; i < numberOfStepsToReturn; i++)
      {
         ret.add(stepList.get(i).getFramePointCopy());
      }

      return ret;
   }

   private ArrayList<YoFramePoint> createABunchOfUniformWalkingSteps(RobotSide stepSide, boolean startSquaredUp, int numberOfStepsInStepList,
           double stepLength, double halfStepWidth)
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


   private ArrayList<YoFramePoint> createABunchOfRandomWalkingSteps(RobotSide stepSide, boolean startSquaredUp, int numberOfStepsInStepList, double stepLength,
           double halfStepWidth)
   {
      Random random = new Random(1776L);
      ArrayList<YoFramePoint> footLocations = new ArrayList<YoFramePoint>();

      double height = 0.5;

      Point3d previousStepLocation = new Point3d(0.0, 0.0, height);


      if (startSquaredUp)
      {
         YoFramePoint firstStepLocation = new YoFramePoint("stepListElementFirstSquaredUpStep", "", ReferenceFrame.getWorldFrame(), registry);
         firstStepLocation.set(-0.5, stepSide.negateIfRightSide(halfStepWidth), height);
         footLocations.add(firstStepLocation);

         stepSide = stepSide.getOppositeSide();
         firstStepLocation.get(previousStepLocation);

         YoFramePoint secondStepLocation = new YoFramePoint("stepListElementSecondSquaredUpStep", "", ReferenceFrame.getWorldFrame(), registry);
         secondStepLocation.set(-0.5, stepSide.negateIfRightSide(halfStepWidth), height);
         footLocations.add(secondStepLocation);

         stepSide = stepSide.getOppositeSide();
         secondStepLocation.get(previousStepLocation);
      }

      double headingDirection = 0.0;

      for (int i = 0; i < numberOfStepsInStepList; i++)
      {
         YoFramePoint stepLocation = new YoFramePoint("stepListElement" + i, "", ReferenceFrame.getWorldFrame(), registry);

         double headingOffset = RandomTools.generateRandomDouble(random, -Math.PI / 4.0, Math.PI / 4.0);
         headingDirection = headingDirection + headingOffset;

         Vector3d stepOffset = new Vector3d(stepLength, stepSide.negateIfRightSide(halfStepWidth), 0.0);
         Matrix3d rotationMatrix = new Matrix3d();
         rotationMatrix.rotZ(headingDirection);
         rotationMatrix.transform(stepOffset);

         Point3d nextStepLocation = new Point3d(previousStepLocation);
         nextStepLocation.add(stepOffset);

         stepLocation.set(nextStepLocation);
         footLocations.add(stepLocation);
         stepSide = stepSide.getOppositeSide();

         previousStepLocation.set(nextStepLocation);
      }

      return footLocations;
   }

   private void simulateForwardAndCheckSingleSupport(Point3d icpPositionToPack, Vector3d icpVelocityToPack, Point3d ecmpPositionToPack,
           SmoothICPComputer smoothICPComputer, double singleSupportDuration, double initialTime, double omega0, Point3d initialICPPosition,
           Point3d transferFromFoot, ArrayList<YoFramePoint> stepList, ArrayList<FramePoint> footLocations)
   {
      double deltaT = 0.001;
      Point3d previousICPPosition = new Point3d(initialICPPosition);
      for (double time = initialTime + deltaT; time <= initialTime + singleSupportDuration; time = time + deltaT)
      {
         stopSignalFlag.set(timeYoVariable.getDoubleValue() >= stopSignalTime.getDoubleValue());

         if (stopSignalFlag.getBooleanValue())
         {
            removeAllStepsFromStepListExceptFirstTwo(stepList);
            footLocations.clear();
            footLocations.addAll(getFirstSteps(stepList, 4));
            
            for (int i = 1; i < footstepYoFramePoints.size(); i++)
            {
               footstepYoFramePoints.get(i).set(footLocations.get(Math.min(footLocations.size(), footstepYoFramePoints.size())-1));
            }
         }


         smoothICPComputer.getICPPositionAndVelocity(icpPositionToPack, icpVelocityToPack, ecmpPositionToPack, time);

         if (visualize)
         {
            visualizeICPAndECMP(icpPositionToPack, icpVelocityToPack, ecmpPositionToPack, time);
         }

         if (USE_ASSERTS)
         {
            assertICPPointsAreAlongLineIncludingTransferFromFoot(transferFromFoot, initialICPPosition, icpPositionToPack);

            Vector3d approximateVelocity = new Vector3d(icpPositionToPack);
            approximateVelocity.sub(previousICPPosition);
            approximateVelocity.scale(1.0 / deltaT);

            JUnitTools.assertTuple3dEquals(approximateVelocity, icpVelocityToPack, 3e-3);
         }

         previousICPPosition.set(icpPositionToPack);
      }

      if (true)
      {
         int debugger = 1;
      }
   }

   private void simulateForwardAndCheckDoubleSupport(Point3d icpPositionToPack, Vector3d icpVelocityToPack, Point3d ecmpPositionToPack,
           SmoothICPComputer smoothICPComputer, double doubleSupportDuration, double initialTime, double deltaT, double omega0, Point3d initialICPPosition,
           ArrayList<YoFramePoint> stepList, ArrayList<FramePoint> footLocations)
   {
      Point3d previousICPPosition = new Point3d(initialICPPosition);
      for (double time = initialTime + deltaT; time <= initialTime + doubleSupportDuration; time = time + deltaT)
      {
         stopSignalFlag.set(timeYoVariable.getDoubleValue() >= stopSignalTime.getDoubleValue());

         if (stopSignalFlag.getBooleanValue())
         {
            removeAllStepsFromStepListExceptFirstTwo(stepList);
            footLocations.clear();
            footLocations.addAll(getFirstSteps(stepList, 4));
            
            for (int i = 1; i < footstepYoFramePoints.size(); i++)
            {
               footstepYoFramePoints.get(i).set(footLocations.get(Math.min(footLocations.size(), footstepYoFramePoints.size())-1));
            }
         }


         smoothICPComputer.getICPPositionAndVelocity(icpPositionToPack, icpVelocityToPack, ecmpPositionToPack, time);

         if (visualize)
         {
            visualizeICPAndECMP(icpPositionToPack, icpVelocityToPack, ecmpPositionToPack, time);
         }

         if (USE_ASSERTS)
         {
            Vector3d approximateVelocity = new Vector3d(icpPositionToPack);
            approximateVelocity.sub(previousICPPosition);
            approximateVelocity.scale(1.0 / deltaT);

            JUnitTools.assertTuple3dEquals(approximateVelocity, icpVelocityToPack, 3e-3);
         }

         previousICPPosition.set(icpPositionToPack);
      }
   }


   public void visualizeICPAndECMP(Point3d icpPosition, Vector3d icpVelocity, Point3d ecmpPosition, double time)
   {
      icpPositionYoFramePoint.set(icpPosition);
      icpVelocityYoFramePoint.set(icpVelocity);

      PointAndLinePlotter.setEndPointGivenStartAndAdditionalVector(icpArrowTip, icpPosition, icpVelocity, 0.5);
      Point2d icpPosition2d = new Point2d(icpPosition.getX(), icpPosition.getY());
      PointAndLinePlotter.setLineSegmentBasedOnStartAndEndFramePoints(icpVelocityLineSegment, icpPosition2d, icpArrowTip.getFramePoint2dCopy().getPointCopy());

      ecmpPositionYoFramePoint.set(ecmpPosition);

      timeYoVariable.set(time);
      scs.tickAndUpdate();
   }


   private void assertICPPointsAreAlongLineIncludingTransferFromFoot(Point3d transferFromFoot, Point3d initialICPPosition, Point3d icpPosition)
   {
      boolean pointsInOrderAndColinear = GeometryTools.arePointsInOrderAndColinear(transferFromFoot, initialICPPosition, icpPosition, 1e-4);

      if (!pointsInOrderAndColinear)
      {
         fail();
      }
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testTrivialTwoStepExample()
   {
      int maxNumberOfConsideredFootsteps = 4;
      double doubleSupportFirstStepFraction = 0.55;
      SmoothICPComputer smoothICPComputer = new SmoothICPComputer(doubleSupportFirstStepFraction, maxNumberOfConsideredFootsteps, registry,
                                               yoGraphicsListRegistry);

      ArrayList<FramePoint> footLocations = new ArrayList<FramePoint>();
      footLocations.add(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0));
      footLocations.add(new FramePoint(ReferenceFrame.getWorldFrame(), 1.0, 1.0, 0.0));

      double singleSupportDuration = 0.5;
      double doubleSupportDuration = 0.2;
      double doubleSupportInitialTransferDuration = 0.3;

      double initialTime = 1.0;
      double omega0 = 0.33;

      boolean stopIfReachedEnd = true;

      Point3d initialICPPosition = new Point3d();

      initialICPPosition.set(footLocations.get(0).getPoint());
      initialICPPosition.add(footLocations.get(1).getPoint());
      initialICPPosition.scale(0.5);

      smoothICPComputer.initializeDoubleSupportInitialTransfer(footLocations, initialICPPosition, singleSupportDuration, doubleSupportDuration,
              doubleSupportInitialTransferDuration, omega0, initialTime, stopIfReachedEnd);

      Point3d icpPositionToPack = new Point3d();
      Vector3d icpVelocityToPack = new Vector3d();
      Point3d ecmpPositionToPack = new Point3d();

      double time = initialTime;
      smoothICPComputer.getICPPositionAndVelocity(icpPositionToPack, icpVelocityToPack, ecmpPositionToPack, time);

      Point3d averagePoint = new Point3d(footLocations.get(0).getPoint());
      averagePoint.add(footLocations.get(1).getPoint());

      averagePoint.scale(0.5);

      if (USE_ASSERTS)
      {
         JUnitTools.assertTuple3dEquals(averagePoint, icpPositionToPack, 1e-7);
         JUnitTools.assertTuple3dEquals(new Vector3d(), icpVelocityToPack, 1e-7);
      }

      time = initialTime + doubleSupportInitialTransferDuration;
      smoothICPComputer.getICPPositionAndVelocity(icpPositionToPack, icpVelocityToPack, ecmpPositionToPack, time);

      if (USE_ASSERTS)
      {
         JUnitTools.assertTuple3dEquals(averagePoint, icpPositionToPack, 1e-7);
         JUnitTools.assertTuple3dEquals(new Vector3d(), icpVelocityToPack, 1e-7);
      }
   }


   private void createVisualizers(int maxNumberOfConsideredFootsteps, boolean visualize)
   {
//      visualize = true;
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      Robot robot = new Robot("TestRobot");
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setShowSplashScreen(visualize);
      parameters.setShowWindows(visualize);
      
      scs = new SimulationConstructionSet(robot, parameters);

      scs.setDT(1e-3, 1);
      scs.changeBufferSize(16000);
      scs.setPlaybackRealTimeRate(0.2);
      scs.setPlaybackDesiredFrameRate(0.001);

      robot.getRobotsYoVariableRegistry().addChild(registry);
      timeYoVariable = robot.getYoTime();

      pointAndLinePlotter = new PointAndLinePlotter(registry);
      pointAndLinePlotter.createAndShowOverheadPlotterInSCS(scs);

      icpArrowTip = new YoFramePoint("icpVelocityTip", "", worldFrame, registry);
      icpVelocityLineSegment = new YoFrameLineSegment2d("icpVelocity", "", worldFrame, registry);
      icpPositionYoFramePoint = new YoFramePoint("icpPosition", "", worldFrame, registry);
      icpVelocityYoFramePoint = new YoFramePoint("icpVelocity", "", worldFrame, registry);
      ecmpPositionYoFramePoint = new YoFramePoint("ecmpPosition", "", worldFrame, registry);

      doubleSupportStartICPYoFramePoint = new YoFramePoint("doubleSupportICPStart", "", worldFrame, registry);
      doubleSupportEndICPYoFramePoint = new YoFramePoint("doubleSupportICPEnd", "", worldFrame, registry);


      footstepYoFramePoints = new ArrayList<YoFramePoint>();

      for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
      {
         YoFramePoint footstepYoFramePoint = pointAndLinePlotter.plotPoint3d("footstep" + i, new Point3d(), YoAppearance.Black(), 0.01);
         footstepYoFramePoints.add(footstepYoFramePoint);
      }

      pointAndLinePlotter.plotYoFramePoint("icpPosition", icpPositionYoFramePoint, YoAppearance.OrangeRed(), 0.01);
      pointAndLinePlotter.plotYoFramePoint("ecmpPosition", ecmpPositionYoFramePoint, YoAppearance.Magenta(), 0.011);
      pointAndLinePlotter.plotLineSegment("icpVelocity", icpVelocityLineSegment, Color.gray);

      pointAndLinePlotter.plotYoFramePoint("doubleSupportICPStart", doubleSupportStartICPYoFramePoint, YoAppearance.Cyan(), 0.01);
      pointAndLinePlotter.plotYoFramePoint("doubleSupportICPEnd", doubleSupportEndICPYoFramePoint, YoAppearance.Cyan(), 0.01);

      yoGraphicsListRegistry = pointAndLinePlotter.getDynamicGraphicObjectsListRegistry();
   }


}
