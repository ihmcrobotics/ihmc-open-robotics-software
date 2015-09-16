package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;


import java.awt.Color;
import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.frames.YoFrameLineSegment2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootCenterToToeEvaluator
{
   private boolean visualize = false;

   private PointAndLinePlotter pointAndLinePlotter = null;
   private YoGraphicsListRegistry yoGraphicsListRegistry = null;
   private SimulationConstructionSet scs = null;
   private DoubleYoVariable timeYoVariable = null;


   private YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private YoFramePoint icpArrowTip = null;
   private YoFramePoint icpPositionYoFramePoint = null;
   private YoFramePoint icpVelocityYoFramePoint = null;
   private YoFramePoint ecmpPositionYoFramePoint = null;

   private ArrayList<YoFramePoint> footstepYoFramePoints = null;
   
   
   private ArrayList<YoFramePoint> toePointFramePoints = null;
   
   private ArrayList<YoFramePoint> icpFootCenterCornerPointsViz = null; 
   private ArrayList<YoFramePoint> icpToeCornerPointsViz = null; 
   
   private ArrayList<YoFramePoint> constantCoPsViz = null; 
   

   private YoFramePoint doubleSupportStartICPYoFramePoint = null;
   private YoFramePoint doubleSupportEndICPYoFramePoint = null;

   private final double deltaT = 0.001;

   private double doubleSupportFirstStepFraction = 0.5;
   private int maxNumberOfConsideredFootsteps = 3;
   private DoubleSupportFootCenterToToeICPComputer smoothICPComputer;  


   private YoFrameLineSegment2d icpVelocityLineSegment = null;

   @Before
   public void createRegistryBeforeTests()
   {
      registry = new YoVariableRegistry(getClass().getSimpleName());
      smoothICPComputer = new DoubleSupportFootCenterToToeICPComputer(deltaT, doubleSupportFirstStepFraction,  
            maxNumberOfConsideredFootsteps, registry, yoGraphicsListRegistry);
   }

   private DoubleYoVariable stopSignalTime = new DoubleYoVariable("stopSignalTime", registry);
   private BooleanYoVariable stopSignalFlag = new BooleanYoVariable("stopSignalFlag", registry);

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
      
      toePointFramePoints = null; 
      icpFootCenterCornerPointsViz = null;
      icpToeCornerPointsViz = null; 

      constantCoPsViz = null;

      doubleSupportStartICPYoFramePoint = null;
      doubleSupportEndICPYoFramePoint = null;

      ecmpPositionYoFramePoint = null;
      icpVelocityLineSegment = null;
   }

   public void testTypicalFourStepExampleWithSuddenStop()
   {
      stopSignalTime.set(1.9e100);

      createVisualizers(maxNumberOfConsideredFootsteps);


      RobotSide stepSide = RobotSide.LEFT;
      int numberOfStepsInStepList = 7;
      double stepLength = 0.3;
      double halfStepWidth = 0.1;
      boolean startSquaredUp = true;

      double singleSupportDuration = 1;
      double doubleSupportDuration = 1;
      double doubleSupportInitialTransferDuration = 1;

      double eCMPHeight = 1.0;
      double gravitationalAcceleration = 9.81;

      double omega0 = Math.sqrt(gravitationalAcceleration / eCMPHeight);

      ArrayList<YoFramePoint> stepList = createABunchOfUniformWalkingSteps(stepSide, startSquaredUp, numberOfStepsInStepList, stepLength, halfStepWidth);

//      ArrayList<YoFramePoint> stepList = createABunchOfRandomWalkingSteps(stepSide, startSquaredUp, numberOfStepsInStepList, stepLength, halfStepWidth);

      Point3d initialICPPosition = new Point3d();
      Point3d initialECMPPosition = new Point3d();
      Vector3d initialICPVelocity = new Vector3d();
      Vector3d initialICPAcceleration = new Vector3d();

      Point3d icpPosition = new Point3d();
      Vector3d icpVelocity = new Vector3d();
      Vector3d icpAcceleration = new Vector3d();
      Point3d ecmpPosition = new Point3d();

      double initialTime = 0.0;
      ArrayList<FramePoint> footLocations = getFirstSteps(stepList, 4); 
      ArrayList<ReferenceFrame> dummyReferenceFrameList = new ArrayList<ReferenceFrame>(); 
      
      if (visualize)
      {
         for (int i = 0; i < Math.min(footLocations.size(), footstepYoFramePoints.size()); i++)
         {
            footstepYoFramePoints.get(i).set(footLocations.get(i));
            
            FramePoint tempPoint = new FramePoint(footstepYoFramePoints.get(i).getReferenceFrame()); 
            tempPoint.set(0.1, 0.0, 0.0);
            tempPoint.add(footstepYoFramePoints.get(i).getFramePointCopy());
            toePointFramePoints.get(i).set(tempPoint);
         }
      }

      initialICPPosition.set(footLocations.get(0).getPoint());
      initialICPPosition.add(footLocations.get(1).getPoint());
      initialICPPosition.scale(0.5);
      
      dummyReferenceFrameList.clear();
      for (int i = 0; i < footLocations.size(); i++)
      {
         dummyReferenceFrameList.add(ReferenceFrame.getWorldFrame()); 
      }

      smoothICPComputer.initializeDoubleSupportInitialTransfer(footLocations, dummyReferenceFrameList, initialICPPosition, singleSupportDuration, doubleSupportDuration,
              doubleSupportInitialTransferDuration, omega0, initialTime);
      smoothICPComputer.computeICPPositionVelocityAcceleration(initialICPPosition, initialICPVelocity, initialICPAcceleration, initialECMPPosition, initialTime);

      if (visualize)
      {
         doubleSupportStartICPYoFramePoint.set(smoothICPComputer.getDoubleSupportStartICP());
         doubleSupportEndICPYoFramePoint.set(smoothICPComputer.getDoubleSupportEndICP());
         
         
         
         for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
         {
            constantCoPsViz.get(i).set(smoothICPComputer.getConstantCentersOfPressure().get(i).getFramePointCopy()); 
         }
         
         for (int i = 0; i < maxNumberOfConsideredFootsteps-1; i++)
         {
            icpFootCenterCornerPointsViz.get(i).set(smoothICPComputer.getFootCenterICPCornerPoints().get(i).getFramePointCopy()); 
            
            if (smoothICPComputer.getDoHeelToToeTransfer())
            {
            icpToeCornerPointsViz.get(i).set(smoothICPComputer.getToeICPCornerPoints().get(i).getFramePointCopy()); 
            }
         }
      }

      Point3d transferFromFoot = footLocations.get(0).getPoint();
      simulateForwardAndCheckDoubleSupport(icpPosition, icpVelocity, icpAcceleration, ecmpPosition, smoothICPComputer, doubleSupportInitialTransferDuration, initialTime,
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
            
            FramePoint tempPoint = new FramePoint(footstepYoFramePoints.get(i).getReferenceFrame()); 
            tempPoint.set(0.1, 0.0, 0.0);
            tempPoint.add(footstepYoFramePoints.get(i).getFramePointCopy());
            toePointFramePoints.get(i).set(tempPoint);
         }
      }

      if (visualize)
      {
         doubleSupportStartICPYoFramePoint.set(smoothICPComputer.getDoubleSupportStartICP());
         doubleSupportEndICPYoFramePoint.set(smoothICPComputer.getDoubleSupportEndICP());
         
         for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
         {
            constantCoPsViz.get(i).set(smoothICPComputer.getConstantCentersOfPressure().get(i).getFramePointCopy()); 
         }
         
         for (int i = 0; i < maxNumberOfConsideredFootsteps-1; i++)
         {
            icpFootCenterCornerPointsViz.get(i).set(smoothICPComputer.getFootCenterICPCornerPoints().get(i).getFramePointCopy()); 
            
            if (smoothICPComputer.getDoHeelToToeTransfer())
            {
            icpToeCornerPointsViz.get(i).set(smoothICPComputer.getToeICPCornerPoints().get(i).getFramePointCopy()); 
            }
         }
      }

      while (stepList.size() >= 2)
      {
         footLocations = getFirstSteps(stepList, 4);

         if (visualize)
         {
            for (int i = 0; i < Math.min(footLocations.size(), footstepYoFramePoints.size()); i++)
            {
               footstepYoFramePoints.get(i).set(footLocations.get(i));
               
               FramePoint tempPoint = new FramePoint(footstepYoFramePoints.get(i).getReferenceFrame()); 
               tempPoint.set(0.1, 0.0, 0.0);
               tempPoint.add(footstepYoFramePoints.get(i).getFramePointCopy());
               toePointFramePoints.get(i).set(tempPoint);
            }
         }

         
         dummyReferenceFrameList.clear(); 
         for (int i = 0; i < footLocations.size(); i++)
         {
            dummyReferenceFrameList.add(ReferenceFrame.getWorldFrame()); 
         }
         
         smoothICPComputer.initializeSingleSupport(footLocations, dummyReferenceFrameList, singleSupportDuration, doubleSupportDuration, omega0, initialTime);

         if (visualize)
         {
            doubleSupportStartICPYoFramePoint.set(smoothICPComputer.getDoubleSupportStartICP());
            doubleSupportEndICPYoFramePoint.set(smoothICPComputer.getDoubleSupportEndICP());
            
            for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
            {
               constantCoPsViz.get(i).set(smoothICPComputer.getConstantCentersOfPressure().get(i).getFramePointCopy()); 
            }
            
            for (int i = 0; i < maxNumberOfConsideredFootsteps-1; i++)
            {
               icpFootCenterCornerPointsViz.get(i).set(smoothICPComputer.getFootCenterICPCornerPoints().get(i).getFramePointCopy()); 
               
               if (smoothICPComputer.getDoHeelToToeTransfer())
               {
               icpToeCornerPointsViz.get(i).set(smoothICPComputer.getToeICPCornerPoints().get(i).getFramePointCopy()); 
               }
            }
         }

         smoothICPComputer.computeICPPositionVelocityAcceleration(initialICPPosition, initialICPVelocity, initialICPAcceleration, initialECMPPosition, initialTime);


         transferFromFoot = footLocations.get(0).getPoint();

         simulateForwardAndCheckSingleSupport(icpPosition, icpVelocity, icpAcceleration, ecmpPosition, smoothICPComputer, singleSupportDuration, initialTime, omega0,
                 initialICPPosition, transferFromFoot, stepList, footLocations);

         initialTime = initialTime + singleSupportDuration;

         dummyReferenceFrameList.clear(); 
         for (int i = 0; i < footLocations.size(); i++)
         {
            dummyReferenceFrameList.add(ReferenceFrame.getWorldFrame()); 
         }
         
         smoothICPComputer.initializeDoubleSupport(footLocations, dummyReferenceFrameList, singleSupportDuration, doubleSupportDuration, omega0, null, null, initialTime);
         smoothICPComputer.computeICPPositionVelocityAcceleration(initialICPPosition, initialICPVelocity, initialICPAcceleration, initialECMPPosition, initialTime);


         if (visualize)
         {
            doubleSupportStartICPYoFramePoint.set(smoothICPComputer.getDoubleSupportStartICP());
            doubleSupportEndICPYoFramePoint.set(smoothICPComputer.getDoubleSupportEndICP());
            
            for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
            {
               constantCoPsViz.get(i).set(smoothICPComputer.getConstantCentersOfPressure().get(i).getFramePointCopy()); 
            }
            
            for (int i = 0; i < maxNumberOfConsideredFootsteps-1; i++)
            {
               icpFootCenterCornerPointsViz.get(i).set(smoothICPComputer.getFootCenterICPCornerPoints().get(i).getFramePointCopy()); 
               
               if (smoothICPComputer.getDoHeelToToeTransfer())
               {
               icpToeCornerPointsViz.get(i).set(smoothICPComputer.getToeICPCornerPoints().get(i).getFramePointCopy()); 
               }
            }
         }

         simulateForwardAndCheckDoubleSupport(icpPosition, icpVelocity, icpAcceleration, ecmpPosition, smoothICPComputer, doubleSupportDuration, initialTime, deltaT, omega0,
                 initialICPPosition, stepList, footLocations);

         initialTime = initialTime + doubleSupportDuration;


         stepList.remove(0);
      }

      double timeToSimulateAtEnd = 1.0;
      initialICPPosition.set(icpPosition);
      simulateForwardAndCheckDoubleSupport(icpPosition, icpVelocity, icpAcceleration, ecmpPosition, smoothICPComputer, timeToSimulateAtEnd, initialTime, deltaT, omega0,
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

      for (int i = stepListSize - 1; i >= 2; i--)
      {
         stepList.remove(i);
      }
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

   private void simulateForwardAndCheckSingleSupport(Point3d icpPositionToPack, Vector3d icpVelocityToPack, Vector3d icpAccelerationToPack, Point3d ecmpPositionToPack,
         DoubleSupportFootCenterToToeICPComputer smoothICPComputer, double singleSupportDuration, double initialTime, double omega0, Point3d initialICPPosition,
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
               
               FramePoint tempPoint = new FramePoint(footstepYoFramePoints.get(i).getReferenceFrame()); 
               tempPoint.set(0.1, 0.0, 0.0);
               tempPoint.add(footstepYoFramePoints.get(i).getFramePointCopy());
               toePointFramePoints.get(i).set(tempPoint);
            }
            
            for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
            {
               constantCoPsViz.get(i).set(smoothICPComputer.getConstantCentersOfPressure().get(i).getFramePointCopy()); 
            }
            
            for (int i = 0; i < maxNumberOfConsideredFootsteps-1; i++)
            {
               icpFootCenterCornerPointsViz.get(i).set(smoothICPComputer.getFootCenterICPCornerPoints().get(i).getFramePointCopy()); 
               
               if (smoothICPComputer.getDoHeelToToeTransfer())
               {
               icpToeCornerPointsViz.get(i).set(smoothICPComputer.getToeICPCornerPoints().get(i).getFramePointCopy()); 
               }
            }
         }


         smoothICPComputer.computeICPPositionVelocityAcceleration(icpPositionToPack, icpVelocityToPack, icpAccelerationToPack, ecmpPositionToPack, time);

         if (visualize)
         {
            visualizeICPAndECMP(icpPositionToPack, icpVelocityToPack, ecmpPositionToPack, time);
         }

         previousICPPosition.set(icpPositionToPack);
      }
   }

   private void simulateForwardAndCheckDoubleSupport(Point3d icpPositionToPack, Vector3d icpVelocityToPack, Vector3d icpAccelerationToPack, Point3d ecmpPositionToPack,
         DoubleSupportFootCenterToToeICPComputer smoothICPComputer, double doubleSupportDuration, double initialTime, double deltaT, double omega0, Point3d initialICPPosition,
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
               
               FramePoint tempPoint = new FramePoint(footstepYoFramePoints.get(i).getReferenceFrame()); 
               tempPoint.set(0.1, 0.0, 0.0);
               tempPoint.add(footstepYoFramePoints.get(i).getFramePointCopy());
               toePointFramePoints.get(i).set(tempPoint);
            }
            
            for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
            {
               constantCoPsViz.get(i).set(smoothICPComputer.getConstantCentersOfPressure().get(i).getFramePointCopy()); 
            }
            
            for (int i = 0; i < maxNumberOfConsideredFootsteps-1; i++)
            {
               icpFootCenterCornerPointsViz.get(i).set(smoothICPComputer.getFootCenterICPCornerPoints().get(i).getFramePointCopy()); 
               
               if (smoothICPComputer.getDoHeelToToeTransfer())
               {
               icpToeCornerPointsViz.get(i).set(smoothICPComputer.getToeICPCornerPoints().get(i).getFramePointCopy()); 
               }
            }
         }


         smoothICPComputer.computeICPPositionVelocityAcceleration(icpPositionToPack, icpVelocityToPack, icpAccelerationToPack, ecmpPositionToPack, time);

         if (visualize)
         {
            visualizeICPAndECMP(icpPositionToPack, icpVelocityToPack, ecmpPositionToPack, time);
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


      footstepYoFramePoints = new ArrayList<YoFramePoint>();
      toePointFramePoints = new ArrayList<YoFramePoint>();
      icpFootCenterCornerPointsViz = new ArrayList<YoFramePoint>();
      icpToeCornerPointsViz = new ArrayList<YoFramePoint>();
      constantCoPsViz = new ArrayList<YoFramePoint>();
      
      for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
      {
         YoFramePoint footstepYoFramePoint = pointAndLinePlotter.plotPoint3d("footstep" + i, new Point3d(), YoAppearance.Black(), 0.003);
         footstepYoFramePoints.add(footstepYoFramePoint);
         
         YoFramePoint toeFramePoint = pointAndLinePlotter.plotPoint3d("toePoint" + i, new Point3d(), YoAppearance.Black(), 0.003);
         toePointFramePoints.add(toeFramePoint);
      }
      
      for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
      {
         YoFramePoint constantCoPPoint = pointAndLinePlotter.plotPoint3d("constantCoPsViz" + i, new Point3d(), YoAppearance.Red(), 0.004);
         constantCoPsViz.add(constantCoPPoint); 
      }
      
      
      for (int i = 0; i < maxNumberOfConsideredFootsteps - 1; i++)
      {
         YoFramePoint cornerICPFramePoint = pointAndLinePlotter.plotPoint3d("cornerICPFramePoint" + i, new Point3d(), YoAppearance.Green(), 0.003);
         icpFootCenterCornerPointsViz.add(cornerICPFramePoint);
         
         if (smoothICPComputer.getDoHeelToToeTransfer())
         {
            YoFramePoint toeCornerICPFramePoint = pointAndLinePlotter.plotPoint3d("toeCornerICPFramePoint" + i, new Point3d(), YoAppearance.Blue(), 0.003);
            icpToeCornerPointsViz.add(toeCornerICPFramePoint);
         }
      }
      
    

      pointAndLinePlotter.plotYoFramePoint("icpPosition", icpPositionYoFramePoint, YoAppearance.OrangeRed(), 0.003);
      pointAndLinePlotter.plotYoFramePoint("ecmpPosition", ecmpPositionYoFramePoint, YoAppearance.Magenta(), 0.005);
      pointAndLinePlotter.plotLineSegment("icpVelocity", icpVelocityLineSegment, Color.gray);

      pointAndLinePlotter.plotYoFramePoint("doubleSupportICPStart", doubleSupportStartICPYoFramePoint, YoAppearance.Cyan(), 0.003);
      pointAndLinePlotter.plotYoFramePoint("doubleSupportICPEnd", doubleSupportEndICPYoFramePoint, YoAppearance.Cyan(), 0.004);

      yoGraphicsListRegistry = pointAndLinePlotter.getDynamicGraphicObjectsListRegistry();
   }


}
