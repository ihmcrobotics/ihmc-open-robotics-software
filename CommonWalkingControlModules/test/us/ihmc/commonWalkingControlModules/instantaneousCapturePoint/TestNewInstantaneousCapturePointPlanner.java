package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.awt.Color;
import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;

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

   private PointAndLinePlotter pointAndLinePlotter = new PointAndLinePlotter(registry);
   private YoGraphicsListRegistry yoGraphicsListRegistry = null;
   private SimulationConstructionSet scs = null;
   private DoubleYoVariable timeYoVariable = null;
   private DoubleYoVariable stopSignalTime = new DoubleYoVariable("stopSignalTime", registry);
   private BooleanYoVariable stopSignalFlag = new BooleanYoVariable("stopSignalFlag", registry);
   
   YoFramePoint tmpPreviousICPPosition = new YoFramePoint("PreviousICPPosition", ReferenceFrame.getWorldFrame(), registry);

   private YoFramePoint icpArrowTip = null;
   private YoFramePoint icpPositionYoFramePoint = null;
   private YoFramePoint icpVelocityYoFramePoint = null;
   private YoFramePoint cmpPositionYoFramePoint = null;

   private ArrayList<YoFramePoint> footstepYoFramePoints = null;
   
   private ArrayList<YoFramePoint> icpFootCenterCornerPointsViz = null; 
   
   private ArrayList<YoFramePoint> constantCoPsViz = null; 
   

   private YoFramePoint doubleSupportStartICPYoFramePoint = null;
   private YoFramePoint doubleSupportEndICPYoFramePoint = null;

   private final double deltaT = 0.001;

   private double doubleSupportFirstStepFraction = 0.5;
   private double doubleSupportInitialTransferDuration = 1.0;
   private int numberOfStepsInStepList = 7;
   private int maxNumberOfConsideredFootsteps = 7;  
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
   public void testTypicalFourStepExampleWithSuddenStop()
   {
      icpPlanner = new NewInstantaneousCapturePointPlanner(maxNumberOfConsideredFootsteps,doubleSupportInitialTransferDuration,
            registry,yoGraphicsListRegistry);
      
      stopSignalTime.set(1.9e100);

      createVisualizers(maxNumberOfConsideredFootsteps);

      RobotSide stepSide = RobotSide.LEFT;
      double stepLength = 0.3;
      double halfStepWidth = 0.1;
      boolean startSquaredUp = true;

      double singleSupportDuration = 0.2;
      double doubleSupportDuration = 0.1;
      double doubleSupportInitialTransferDuration = 0.4;

      double comHeight = 1.0;
      double gravitationalAcceleration = 9.81;

      double omega0 = Math.sqrt(gravitationalAcceleration / comHeight);
      

      ArrayList<YoFramePoint> stepList = createABunchOfUniformWalkingSteps(stepSide, startSquaredUp, numberOfStepsInStepList, stepLength, halfStepWidth);

//      ArrayList<YoFramePoint> stepList = createABunchOfRandomWalkingSteps(stepSide, startSquaredUp, numberOfStepsInStepList, stepLength, halfStepWidth);

      YoFramePoint initialICPPosition = new YoFramePoint("initialICP", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint initialCMPPosition = new YoFramePoint("initialCMP", ReferenceFrame.getWorldFrame(), registry);
      YoFrameVector initialICPVelocity = new YoFrameVector("initialICPVelocity", ReferenceFrame.getWorldFrame(), registry);
      YoFrameVector initialICPAcceleration = new YoFrameVector("initialICPAcceleration", ReferenceFrame.getWorldFrame(), registry);

      YoFramePoint icpPosition = new YoFramePoint("icpPosition", ReferenceFrame.getWorldFrame(), registry);
      YoFrameVector icpVelocity = new YoFrameVector("icpVelocity", ReferenceFrame.getWorldFrame(), registry);
      YoFrameVector icpAcceleration = new YoFrameVector("icpAcceleration", ReferenceFrame.getWorldFrame(),registry);
      
      YoFramePoint cmpPosition = new YoFramePoint("cmpPosition", ReferenceFrame.getWorldFrame(), registry);

      double initialTime = 0.0;
      ArrayList<YoFramePoint> footLocations = getFirstSteps(stepList, maxNumberOfConsideredFootsteps);  

      initialICPPosition.set(footLocations.get(0).getPoint3dCopy());
      initialICPPosition.add(footLocations.get(1).getPoint3dCopy());
      initialICPPosition.scale(0.5);

      icpPlanner.initializeDoubleSupport(doubleSupportDuration, singleSupportDuration, initialICPPosition, 
            initialICPVelocity, omega0, initialTime, footLocations);
      icpPlanner.computeDesiredCapturePointPosition(initialTime);
      icpPlanner.computeDesiredCapturePointVelocity(initialTime);
      icpPlanner.computeDesiredCapturePointAcceleration(initialTime);
      
      initialICPPosition.set(icpPlanner.getDesiredCapturePointPosition().getPoint3dCopy());
      initialICPVelocity.set(icpPlanner.getDesiredCapturePointVelocity().getPoint3dCopy());
      initialICPAcceleration.set(icpPlanner.getDesiredCapturePointAcceleration().getPoint3dCopy());

      if (visualize)
      {
         for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
         {
            constantCoPsViz.get(i).set(icpPlanner.getConstantCentersOfPressure().get(i)); 
         }
         
         for (int i = 0; i < maxNumberOfConsideredFootsteps-1; i++)
         {
            icpFootCenterCornerPointsViz.get(i).set(icpPlanner.getCapturePointCornerPoints().get(i)); 
         }
      }

      simulateForwardAndCheckDoubleSupport(icpPosition, icpVelocity, icpAcceleration, cmpPosition, icpPlanner, doubleSupportInitialTransferDuration, initialTime,
              deltaT, omega0, initialICPPosition.getPoint3dCopy(), stepList, footLocations);

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
//         doubleSupportStartICPYoFramePoint.set(icpPlanner.getDoubleSupportStartICP());
//         doubleSupportEndICPYoFramePoint.set(icpPlanner.getDoubleSupportEndICP());
//         
//         for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
//         {
//            constantCoPsViz.get(i).set(icpPlanner.getConstantCentersOfPressure().get(i).getFramePointCopy()); 
//         }
         
         for (int i = 0; i < maxNumberOfConsideredFootsteps-1; i++)
         {
            icpFootCenterCornerPointsViz.get(i).set(icpPlanner.getCapturePointCornerPoints().get(i).getFramePointCopy()); 
         }
      }

//      while (stepList.size() >= 2)
//      {
//         footLocations = getFirstSteps(stepList, 4);
//         
//         icpPlanner.initializeSingleSupport( singleSupportDuration, doubleSupportDuration, omega0, initialTime,footLocations);
//
//         if (visualize)
//         {
////            doubleSupportStartICPYoFramePoint.set(icpPlanner.getDoubleSupportStartICP());
////            doubleSupportEndICPYoFramePoint.set(icpPlanner.getDoubleSupportEndICP());
//            
//            for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
//            {
//               constantCoPsViz.get(i).set(icpPlanner.getConstantCentersOfPressure().get(i).getFramePointCopy()); 
//            }
//            
//            for (int i = 0; i < maxNumberOfConsideredFootsteps-1; i++)
//            {
//               icpFootCenterCornerPointsViz.get(i).set(icpPlanner.getCapturePointCornerPoints().get(i).getFramePointCopy()); 
//            }
//         }
//
//         icpPlanner.computeDesiredCapturePointPosition(initialTime);
//         icpPlanner.computeDesiredCapturePointVelocity(initialTime);
//         icpPlanner.computeDesiredCapturePointAcceleration(initialTime);
//         
//         initialICPPosition = icpPlanner.getDesiredCapturePointPosition();
//         initialICPVelocity = icpPlanner.getDesiredCapturePointVelocity();
//         initialICPAcceleration = icpPlanner.getDesiredCapturePointAcceleration();
//
//         transferFromFoot = footLocations.get(0).getPoint3dCopy();
//
//         simulateForwardAndCheckSingleSupport(icpPosition, icpVelocity, icpAcceleration, cmpPosition, icpPlanner, singleSupportDuration, initialTime, omega0,
//                 initialICPPosition.getPoint3dCopy(), transferFromFoot, stepList, footLocations);
//
//         initialTime = initialTime + singleSupportDuration;
//
//         dummyReferenceFrameList.clear(); 
//         for (int i = 0; i < footLocations.size(); i++)
//         {
//            dummyReferenceFrameList.add(ReferenceFrame.getWorldFrame()); 
//         }
//         
//         smoothICPComputer.initializeDoubleSupport(footLocations, dummyReferenceFrameList, singleSupportDuration, doubleSupportDuration, omega0, null, null, initialTime);
//         smoothICPComputer.computeICPPositionVelocityAcceleration(initialICPPosition, initialICPVelocity, initialICPAcceleration, initialECMPPosition, initialTime);
//
//
//         if (visualize)
//         {
//            doubleSupportStartICPYoFramePoint.set(smoothICPComputer.getDoubleSupportStartICP());
//            doubleSupportEndICPYoFramePoint.set(smoothICPComputer.getDoubleSupportEndICP());
//            
//            for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
//            {
//               constantCoPsViz.get(i).set(smoothICPComputer.getConstantCentersOfPressure().get(i).getFramePointCopy()); 
//            }
//            
//            for (int i = 0; i < maxNumberOfConsideredFootsteps-1; i++)
//            {
//               icpFootCenterCornerPointsViz.get(i).set(smoothICPComputer.getFootCenterICPCornerPoints().get(i).getFramePointCopy()); 
//               
//               if (smoothICPComputer.getDoHeelToToeTransfer())
//               {
//               icpToeCornerPointsViz.get(i).set(smoothICPComputer.getToeICPCornerPoints().get(i).getFramePointCopy()); 
//               }
//            }
//         }
//
//         simulateForwardAndCheckDoubleSupport(icpPosition, icpVelocity, icpAcceleration, ecmpPosition, smoothICPComputer, doubleSupportDuration, initialTime, deltaT, omega0,
//                 initialICPPosition, stepList, footLocations);
//
//         initialTime = initialTime + doubleSupportDuration;
//
//
//         stepList.remove(0);
//      }

      double timeToSimulateAtEnd = 1.0;
      initialICPPosition.set(icpPosition);
      simulateForwardAndCheckDoubleSupport(icpPosition, icpVelocity, icpAcceleration, cmpPosition, icpPlanner, timeToSimulateAtEnd, initialTime, deltaT, omega0,
              initialICPPosition.getPoint3dCopy(), stepList, footLocations);

      if (visualize)
      {
         pointAndLinePlotter.addPointsAndLinesToSCS(scs);

         scs.startOnAThread();
         ThreadTools.sleepForever();
      }
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
   
   private ArrayList<YoFramePoint> getFirstSteps(ArrayList<YoFramePoint> stepList, int maximumNumberOfStepsToReturn)
   {
      ArrayList<YoFramePoint> ret = new ArrayList<YoFramePoint>();

      int numberOfStepsToReturn = Math.min(stepList.size(), maximumNumberOfStepsToReturn);
      for (int i = 0; i < numberOfStepsToReturn; i++)
      {
         ret.add(stepList.get(i));
      }

      return ret;
   }
   
   private void createVisualizers(int maxNumberOfConsideredFootsteps)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      Robot robot = new Robot("TestRobot");
      scs = new SimulationConstructionSet(robot);

      scs.setDT(deltaT, 1);
      scs.changeBufferSize(16000);
      scs.setPlaybackRealTimeRate(scsPlaybackRate );
      scs.setPlaybackDesiredFrameRate(scsPlaybackDesiredFrameRate );

      robot.getRobotsYoVariableRegistry().addChild(registry);
      timeYoVariable = robot.getYoTime();

      pointAndLinePlotter.createAndShowOverheadPlotterInSCS(scs);

      icpArrowTip = new YoFramePoint("icpVelocityTip", "", worldFrame, registry);
      icpVelocityLineSegment = new YoFrameLineSegment2d("icpVelocityForViz", "", worldFrame, registry);
      icpPositionYoFramePoint = new YoFramePoint("icpPositionForViz", "", worldFrame, registry);
      icpVelocityYoFramePoint = new YoFramePoint("icpVelocityForViz", "", worldFrame, registry);
      cmpPositionYoFramePoint = new YoFramePoint("ecmpPositionForViz", "", worldFrame, registry);

      doubleSupportStartICPYoFramePoint = new YoFramePoint("doubleSupportICPStart", "", worldFrame, registry);
      doubleSupportEndICPYoFramePoint = new YoFramePoint("doubleSupportICPEnd", "", worldFrame, registry);

      footstepYoFramePoints = new ArrayList<YoFramePoint>();
      icpFootCenterCornerPointsViz = new ArrayList<YoFramePoint>();
      constantCoPsViz = new ArrayList<YoFramePoint>();
      
      for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
      {
         YoFramePoint footstepYoFramePoint = pointAndLinePlotter.plotPoint3d("footstep" + i, new Point3d(), YoAppearance.Black(), 0.003);
         footstepYoFramePoints.add(footstepYoFramePoint);
         
      }
      
      for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
      {
         YoFramePoint constantCoPPoint = pointAndLinePlotter.plotPoint3d("constantCoPsViz" + i, new Point3d(), YoAppearance.Red(), 0.004);
         constantCoPsViz.add(constantCoPPoint); 
      }
      
      
      for (int i = 0; i < maxNumberOfConsideredFootsteps - 1; i++)
      {
         YoFramePoint cornerICPFramePoint = pointAndLinePlotter.plotPoint3d("icpCornerPoint" + i, new Point3d(), YoAppearance.Green(), 0.003);
         icpFootCenterCornerPointsViz.add(cornerICPFramePoint);
      }

      pointAndLinePlotter.plotYoFramePoint("icpPosition", icpPositionYoFramePoint, YoAppearance.OrangeRed(), 0.003);
      pointAndLinePlotter.plotYoFramePoint("cmpPosition", cmpPositionYoFramePoint, YoAppearance.Magenta(), 0.005);
      pointAndLinePlotter.plotLineSegment("icpVelocity", icpVelocityLineSegment, Color.gray);

      pointAndLinePlotter.plotYoFramePoint("doubleSupportICPStart", doubleSupportStartICPYoFramePoint, YoAppearance.Cyan(), 0.003);
      pointAndLinePlotter.plotYoFramePoint("doubleSupportICPEnd", doubleSupportEndICPYoFramePoint, YoAppearance.Cyan(), 0.004);

      yoGraphicsListRegistry = pointAndLinePlotter.getDynamicGraphicObjectsListRegistry();
   }
   
   private void simulateForwardAndCheckSingleSupport(Point3d icpPositionToPack, Vector3d icpVelocityToPack, Vector3d icpAccelerationToPack, Point3d ecmpPositionToPack,
         NewInstantaneousCapturePointPlanner smoothICPComputer, double singleSupportDuration, double initialTime, double omega0, Point3d initialICPPosition,
           Point3d transferFromFoot, ArrayList<YoFramePoint> stepList, ArrayList<YoFramePoint> footLocations)
   {
      Point3d previousICPPosition = new Point3d(initialICPPosition);
      for (double time = initialTime + deltaT; time <= initialTime + singleSupportDuration; time = time + deltaT)
      {
         stopSignalFlag.set(timeYoVariable.getDoubleValue() >= stopSignalTime.getDoubleValue());

         if (stopSignalFlag.getBooleanValue())
         {
            removeAllStepsFromStepListExceptFirstTwo(stepList);
            footLocations.clear();
            footLocations.addAll(getFirstSteps(stepList, 4));
            
            for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
            {
               constantCoPsViz.get(i).set(smoothICPComputer.getConstantCentersOfPressure().get(i).getFramePointCopy()); 
            }
            
            for (int i = 0; i < maxNumberOfConsideredFootsteps-1; i++)
            {
               icpFootCenterCornerPointsViz.get(i).set(smoothICPComputer.getCapturePointCornerPoints().get(i).getFramePointCopy());
            }
         }

         icpPlanner.computeDesiredCapturePointPosition(time);
         icpPlanner.computeDesiredCapturePointVelocity(time);
         icpPlanner.computeDesiredCapturePointAcceleration(time);
        
         icpPositionToPack.set(icpPlanner.getDesiredCapturePointPosition().getPoint3dCopy());
         icpVelocityToPack.set(icpPlanner.getDesiredCapturePointVelocity().getPoint3dCopy());
         icpAccelerationToPack.set(icpPlanner.getDesiredCapturePointAcceleration().getPoint3dCopy());

         if (visualize)
         {
            visualizeICPAndECMP(icpPositionToPack, icpVelocityToPack, ecmpPositionToPack, time);
         }

         previousICPPosition.set(icpPositionToPack);
      }
   }
   
   private void simulateForwardAndCheckDoubleSupport(YoFramePoint icpPositionToPack, YoFrameVector icpVelocityToPack, YoFrameVector icpAccelerationToPack, YoFramePoint ecmpPositionToPack,
         NewInstantaneousCapturePointPlanner icpPlanner, double doubleSupportDuration, double initialTime, double deltaT, double omega0, Point3d initialICPPosition,
           ArrayList<YoFramePoint> stepList, ArrayList<YoFramePoint> footLocations)
   {
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
            
            for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
            {
               constantCoPsViz.get(i).set(icpPlanner.getConstantCentersOfPressure().get(i).getFramePointCopy()); 
            }
            
            for (int i = 0; i < maxNumberOfConsideredFootsteps-1; i++)
            {
               icpFootCenterCornerPointsViz.get(i).set(icpPlanner.getCapturePointCornerPoints().get(i)); 
            }
         }

         icpPlanner.computeDesiredCapturePointPosition(time);
         icpPlanner.computeDesiredCapturePointVelocity(time);
         icpPlanner.computeDesiredCapturePointAcceleration(time);
        
         icpPositionToPack.set(icpPlanner.getDesiredCapturePointPosition());
         icpVelocityToPack.set(icpPlanner.getDesiredCapturePointVelocity());
         icpAccelerationToPack.set(icpPlanner.getDesiredCapturePointAcceleration());

         if (visualize)
         {
            visualizeICPAndECMP(icpPlanner.getDesiredCapturePointPosition().getPoint3dCopy(), 
                  icpPlanner.getDesiredCapturePointVelocity().getVector3dCopy(), icpPlanner.getDesiredCapturePointAcceleration().getPoint3dCopy(), time);
         }

         tmpPreviousICPPosition.set(icpPositionToPack);
      }
   }
   
   public void visualizeICPAndECMP(Point3d icpPosition, Vector3d icpVelocity, Point3d ecmpPosition, double time)
   {
      icpPositionYoFramePoint.set(icpPosition);
      icpVelocityYoFramePoint.set(icpVelocity);

      PointAndLinePlotter.setEndPointGivenStartAndAdditionalVector(icpArrowTip, icpPosition, icpVelocity, 0.5);
      Point2d icpPosition2d = new Point2d(icpPosition.getX(), icpPosition.getY());
      PointAndLinePlotter.setLineSegmentBasedOnStartAndEndFramePoints(icpVelocityLineSegment, icpPosition2d, icpArrowTip.getFramePoint2dCopy().getPointCopy());

      cmpPositionYoFramePoint.set(ecmpPosition);

      timeYoVariable.set(time);
      scs.tickAndUpdate();
   }
   
   private void removeAllStepsFromStepListExceptFirstTwo(ArrayList<YoFramePoint> stepList)
   {
      int stepListSize = stepList.size();

      for (int i = stepListSize - 1; i >= 2; i--)
      {
         stepList.remove(i);
      }
   }
}
