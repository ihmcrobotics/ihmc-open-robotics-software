package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import static org.junit.Assert.fail;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.math.geometry.GeometryTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.test.JUnitTools;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameLineSegment2d;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

public class SmoothICPComputerTest
{
   private static final boolean USE_ASSERTS = false;
   
   private boolean visualize = false;

   private PointAndLinePlotter pointAndLinePlotter = null;
   private SimulationConstructionSet scs = null;
   private DoubleYoVariable timeYoVariable = null;
   private YoVariableRegistry registry = null;

   private YoFramePoint icpArrowTip = null;
   private YoFramePoint icpPositionYoFramePoint = null;
   private YoFramePoint icpVelocityYoFramePoint = null;
   private YoFramePoint ecmpPositionYoFramePoint = null;

   private ArrayList<YoFramePoint> constantCentersOfPressureYoFramePoints = null;
   
   private YoFramePoint doubleSupportStartICPYoFramePoint = null;
   private YoFramePoint doubleSupportEndICPYoFramePoint = null;


   private YoFrameLineSegment2d icpVelocityLineSegment = null;

   @Before
   public void createRegistryBeforeTests()
   {
      registry = new YoVariableRegistry(getClass().getSimpleName());
   }

   @After
   public void removeVisualizersAfterTest()
   {
      if (scs != null)
      {
         scs.closeAndDispose();
      }

      visualize = false;

      pointAndLinePlotter = null;
      scs = null;
      timeYoVariable = null;
      registry = null;

      icpArrowTip = null;
      icpPositionYoFramePoint = null;
      icpVelocityYoFramePoint = null;
      
      constantCentersOfPressureYoFramePoints = null;
      
      doubleSupportStartICPYoFramePoint = null;
      doubleSupportEndICPYoFramePoint = null;

      ecmpPositionYoFramePoint = null;
      icpVelocityLineSegment = null;
   }

   @Test
   public void testTypicalFourStepExample()
   {
      int maxNumberOfConsideredFootsteps = 4;

      createVisualizers(maxNumberOfConsideredFootsteps);

      double doubleSupportFirstStepFraction = 0.5;
      SmoothICPComputer smoothICPComputer = new SmoothICPComputer(doubleSupportFirstStepFraction, maxNumberOfConsideredFootsteps);

      RobotSide stepSide = RobotSide.LEFT;
      int numberOfStepsInStepList = 10;
      double stepLength = 0.3;
      double halfStepWidth = 0.1;
      boolean startSquaredUp = false;
      boolean stopIfReachedEnd = true;
      
      Random random = new Random(1776L);
      //      ArrayList<YoFramePoint> stepList = createABunchOfUniformWalkingSteps(stepSide, startSquaredUp, numberOfStepsInStepList, stepLength, halfStepWidth);
      ArrayList<YoFramePoint> stepList = createABunchOfRandomWalkingSteps(random, stepSide, startSquaredUp, numberOfStepsInStepList, stepLength, halfStepWidth);

      Point3d initialICPPosition = new Point3d();
      Point3d initialECMPPosition = new Point3d();
      Vector3d initialICPVelocity = new Vector3d();
     
      
      Point3d icpPosition = new Point3d();
      Vector3d icpVelocity = new Vector3d();
      Point3d ecmpPosition = new Point3d();
      
      boolean firstTimeLooping = true;
      
      while(stepList.size() > 3)
      {
         ArrayList<Point3d> footLocations = getFirstSteps(stepList, 4);

         if (visualize)
         {
            pointAndLinePlotter.plotPoint3ds("footLocations", footLocations, YoAppearance.Black(), 0.01);
         }

         double singleSupportDuration = 0.6;
         double doubleSupportDuration = 0.2;

         double initialTime = 1.1;

         double eCMPHeight = 1.0;
         double gravitationalAcceleration = 9.81;

         double omega0 = Math.sqrt(gravitationalAcceleration / eCMPHeight);

         smoothICPComputer.initializeSingleSupport(footLocations, singleSupportDuration, doubleSupportDuration, omega0, initialTime, stopIfReachedEnd);

         if (visualize)
         {
            List<Point3d> constantCentersOfPressure = smoothICPComputer.getConstantCentersOfPressure();
            for (int i=0; i<constantCentersOfPressure.size(); i++)
            {
               constantCentersOfPressureYoFramePoints.get(i).set(constantCentersOfPressure.get(i));
            }
            
            Point3d[] icpCornerPoints = smoothICPComputer.getICPCornerPoints();
            pointAndLinePlotter.plotPoint3ds("icpCornerPoints", icpCornerPoints, YoAppearance.Green(), 0.01);

            doubleSupportStartICPYoFramePoint.set(smoothICPComputer.getDoubleSupportStartICP());
            doubleSupportEndICPYoFramePoint.set(smoothICPComputer.getDoubleSupportEndICP());
         }

         smoothICPComputer.getICPPositionAndVelocity(initialICPPosition, initialICPVelocity, initialECMPPosition, omega0, initialTime);

         if (!firstTimeLooping)
         {
            if (USE_ASSERTS)
            {
               JUnitTools.assertTuple3dEquals(icpPosition, initialICPPosition, 1e-3);
               JUnitTools.assertTuple3dEquals(icpVelocity, initialICPVelocity, 1e-3);
               JUnitTools.assertTuple3dEquals(ecmpPosition, initialECMPPosition, 1e-3);
            }
         }
         
         Point3d transferFromFoot = footLocations.get(0);

         simulateForwardAndCheckSingleSupport(smoothICPComputer, singleSupportDuration, initialTime, omega0, initialICPPosition, transferFromFoot, icpPosition,
               icpVelocity, ecmpPosition);

         initialTime = initialTime + singleSupportDuration;
         smoothICPComputer.initializeDoubleSupport(footLocations, singleSupportDuration, doubleSupportDuration, omega0, initialTime, stopIfReachedEnd);
         smoothICPComputer.getICPPositionAndVelocity(initialICPPosition, initialICPVelocity, initialECMPPosition, omega0, initialTime);

         if (USE_ASSERTS)
         {
            JUnitTools.assertTuple3dEquals(icpPosition, initialICPPosition, 1e-4);
            JUnitTools.assertTuple3dEquals(icpVelocity, initialICPVelocity, 1e-4);
            JUnitTools.assertTuple3dEquals(ecmpPosition, initialECMPPosition, 1e-4);
         }
         
         if (visualize)
         {
            doubleSupportStartICPYoFramePoint.set(smoothICPComputer.getDoubleSupportStartICP());
            doubleSupportEndICPYoFramePoint.set(smoothICPComputer.getDoubleSupportEndICP());
         }

         simulateForwardAndCheckDoubleSupport(smoothICPComputer, doubleSupportDuration, initialTime, omega0, initialICPPosition, transferFromFoot, icpPosition,
               icpVelocity, ecmpPosition);

         initialTime = initialTime + doubleSupportDuration;

         stepList.remove(0);
         firstTimeLooping = false;
      }

      if (visualize)
      {
         pointAndLinePlotter.addPointsAndLinesToSCS(scs);

         scs.startOnAThread();
         ThreadTools.sleepForever();
      }
   }

   private ArrayList<Point3d> getFirstSteps(ArrayList<YoFramePoint> stepList, int numberOfStepsToReturn)
   {
      ArrayList<Point3d> ret = new ArrayList<Point3d>();

      for (int i = 0; i < numberOfStepsToReturn; i++)
      {
         ret.add(stepList.get(i).getFramePointCopy().getPointCopy());
      }

      return ret;
   }

   private ArrayList<Point3d> createFourTypicalSteps()
   {
      ArrayList<Point3d> footLocations = new ArrayList<Point3d>();

      double height = 1.2;

      footLocations.add(new Point3d(0.0, 0.0, height));
      footLocations.add(new Point3d(0.21, 0.22, height + 0.01));
      footLocations.add(new Point3d(0.02, 0.42, height - 0.05));
      footLocations.add(new Point3d(0.2, 0.63, height));

      return footLocations;
   }

   private ArrayList<YoFramePoint> createABunchOfUniformWalkingSteps(RobotSide stepSide, boolean startSquaredUp, int numberOfStepsInStepList, double stepLength,
           double halfStepWidth)
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
   
   
   private ArrayList<YoFramePoint> createABunchOfRandomWalkingSteps(Random random, RobotSide stepSide, boolean startSquaredUp, int numberOfStepsInStepList, double stepLength,
         double halfStepWidth)
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

    Point3d previousStepLocation = new Point3d(0.0, 0.0, height);
    double headingDirection = 0.0;
    
    for (int i = 0; i < numberOfStepsInStepList; i++)
    {
       YoFramePoint stepLocation = new YoFramePoint("stepListElement" + i, "", ReferenceFrame.getWorldFrame(), registry);
       
       double headingOffset = RandomTools.generateRandomDouble(random, -Math.PI/4.0, Math.PI/4.0);
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

   private void simulateForwardAndCheckSingleSupport(SmoothICPComputer smoothICPComputer, double singleSupportDuration, double initialTime, double omega0,
           Point3d initialICPPosition, Point3d transferFromFoot, Point3d icpPosition, Vector3d icpVelocity, Point3d ecmpPosition)
   {
      double deltaT = 0.001;
      Point3d previousICPPosition = new Point3d(initialICPPosition);
      for (double time = initialTime + deltaT; time <= initialTime + singleSupportDuration; time = time + deltaT)
      {
         smoothICPComputer.getICPPositionAndVelocity(icpPosition, icpVelocity, ecmpPosition, omega0, time);

         if (visualize)
         {
            visualizeICPAndECMP(icpPosition, icpVelocity, ecmpPosition, time);
         }

         if (USE_ASSERTS)
         {
            assertICPPointsAreAlongLineIncludingTransferFromFoot(transferFromFoot, initialICPPosition, icpPosition);

            Vector3d approximateVelocity = new Vector3d(icpPosition);
            approximateVelocity.sub(previousICPPosition);
            approximateVelocity.scale(1.0 / deltaT);

            JUnitTools.assertTuple3dEquals(approximateVelocity, icpVelocity, 3e-3);
         }
         
         previousICPPosition.set(icpPosition);
      }
   }

   public void visualizeICPAndECMP(Point3d icpPosition, Vector3d icpVelocity, Point3d ecmpPosition, double time)
   {
      icpPositionYoFramePoint.set(icpPosition);
      icpVelocityYoFramePoint.set(icpVelocity);
      
      PointAndLinePlotter.setEndPointGivenStartAndAdditionalVector(icpArrowTip, icpPosition, icpVelocity, 0.5);
      Point2d icpPosition2d = new Point2d(icpPosition.getX(), icpPosition.getY());
      PointAndLinePlotter.setLineSegmentBasedOnStartAndEndFramePoints(icpVelocityLineSegment, icpPosition2d,
              icpArrowTip.getFramePoint2dCopy().getPointCopy());

      ecmpPositionYoFramePoint.set(ecmpPosition);

      timeYoVariable.set(time);
      scs.tickAndUpdate();
   }

   private void simulateForwardAndCheckDoubleSupport(SmoothICPComputer smoothICPComputer, double doubleSupportDuration, double initialTime, double omega0,
           Point3d initialICPPosition, Point3d transferFromFoot, Point3d icpPosition, Vector3d icpVelocity, Point3d ecmpPosition)
   {
      double deltaT = 0.001;
      Point3d previousICPPosition = new Point3d(initialICPPosition);
      for (double time = initialTime + deltaT; time <= initialTime + doubleSupportDuration; time = time + deltaT)
      {
         smoothICPComputer.getICPPositionAndVelocity(icpPosition, icpVelocity, ecmpPosition, omega0, time);

         if (visualize)
         {
            visualizeICPAndECMP(icpPosition, icpVelocity, ecmpPosition, time);
         }

         if (USE_ASSERTS)
         {
            Vector3d approximateVelocity = new Vector3d(icpPosition);
            approximateVelocity.sub(previousICPPosition);
            approximateVelocity.scale(1.0 / deltaT);

            JUnitTools.assertTuple3dEquals(approximateVelocity, icpVelocity, 3e-3);
         }
         
         previousICPPosition.set(icpPosition);
      }
   }

   private void assertICPPointsAreAlongLineIncludingTransferFromFoot(Point3d transferFromFoot, Point3d initialICPPosition, Point3d icpPosition)
   {
      boolean pointsInOrderAndColinear = GeometryTools.arePointsInOrderAndColinear(transferFromFoot, initialICPPosition, icpPosition, 1e-4);

      if (!pointsInOrderAndColinear)
      {
         fail();
      }
   }

   @Test
   public void testTrivialTwoStepExample()
   {
      int maxNumberOfConsideredFootsteps = 4;
      double doubleSupportFirstStepFraction = 0.55;
      SmoothICPComputer smoothICPComputer = new SmoothICPComputer(doubleSupportFirstStepFraction, maxNumberOfConsideredFootsteps);

      ArrayList<Point3d> footLocations = new ArrayList<Point3d>();
      footLocations.add(new Point3d(0.0, 0.0, 0.0));
      footLocations.add(new Point3d(1.0, 1.0, 0.0));

      double singleSupportDuration = 0.5;
      double doubleSupportDuration = 0.2;
      double doubleSupportInitialTransferDuration = 0.3;

      double initialTime = 1.0;
      double omega0 = 0.33;

      smoothICPComputer.initializeDoubleSupportInitialTransfer(footLocations, singleSupportDuration, doubleSupportDuration,
              doubleSupportInitialTransferDuration, initialTime);

      Point3d icpPositionToPack = new Point3d();
      Vector3d icpVelocityToPack = new Vector3d();
      Point3d ecmpPositionToPack = new Point3d();

      double time = initialTime;
      smoothICPComputer.getICPPositionAndVelocity(icpPositionToPack, icpVelocityToPack, ecmpPositionToPack, omega0, time);

      Point3d averagePoint = new Point3d(footLocations.get(0));
      averagePoint.add(footLocations.get(1));

      averagePoint.scale(0.5);

      if (USE_ASSERTS)
      {
      JUnitTools.assertTuple3dEquals(averagePoint, icpPositionToPack, 1e-7);
      JUnitTools.assertTuple3dEquals(new Vector3d(), icpVelocityToPack, 1e-7);
      }
      
      time = initialTime + doubleSupportInitialTransferDuration;
      smoothICPComputer.getICPPositionAndVelocity(icpPositionToPack, icpVelocityToPack, ecmpPositionToPack, omega0, time);

      if (USE_ASSERTS)
      {
      JUnitTools.assertTuple3dEquals(averagePoint, icpPositionToPack, 1e-7);
      JUnitTools.assertTuple3dEquals(new Vector3d(), icpVelocityToPack, 1e-7);
      }
   }


   private void createVisualizers(int maxNumberOfConsideredFootsteps)
   {
      visualize = true;
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      Robot robot = new Robot("TestRobot");
      scs = new SimulationConstructionSet(robot);

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

      
      constantCentersOfPressureYoFramePoints = new ArrayList<YoFramePoint>();
      for (int i=0; i<maxNumberOfConsideredFootsteps; i++)
      {
         YoFramePoint constantCenterOfPressureYoFramePoint = pointAndLinePlotter.plotPoint3d("constantCoP" + i, new Point3d(), YoAppearance.Red(), 0.005);
         constantCentersOfPressureYoFramePoints.add(constantCenterOfPressureYoFramePoint);
      }

      pointAndLinePlotter.plotYoFramePoint("icpPosition", icpPositionYoFramePoint, YoAppearance.OrangeRed(), 0.01);
      pointAndLinePlotter.plotYoFramePoint("ecmpPosition", ecmpPositionYoFramePoint, YoAppearance.Magenta(), 0.011);
      pointAndLinePlotter.plotLineSegment("icpVelocity", icpVelocityLineSegment, Color.gray);

      pointAndLinePlotter.plotYoFramePoint("doubleSupportICPStart", doubleSupportStartICPYoFramePoint, YoAppearance.Cyan(), 0.01);
      pointAndLinePlotter.plotYoFramePoint("doubleSupportICPEnd", doubleSupportEndICPYoFramePoint, YoAppearance.Cyan(), 0.01);

   }


}
