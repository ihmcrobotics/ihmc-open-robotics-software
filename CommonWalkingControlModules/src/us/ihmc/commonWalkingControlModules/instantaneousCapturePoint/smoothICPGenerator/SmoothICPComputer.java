package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;


public class SmoothICPComputer
{
   private final int maxNumberOfConsideredFootsteps;
   private final double doubleSupportFirstStepFraction;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ArrayList<YoFramePoint> constantCentersOfPressure = new ArrayList<YoFramePoint>();

   private final DoubleYoVariable icpForwardFromCenter = new DoubleYoVariable("icpForwardFromCenter", registry);
   private final DoubleYoVariable icpInFromCenter = new DoubleYoVariable("icpInFromCenter", registry);

   private final BooleanYoVariable hasBeenInitialized = new BooleanYoVariable("icpPlannerHasBeenInitialized", registry);

   private final BooleanYoVariable isDoubleSupport = new BooleanYoVariable("icpPlannerIsDoubleSupport", registry);
   private final DoubleYoVariable timeInState = new DoubleYoVariable("icpPlannerTimeInState", registry);
   private final DoubleYoVariable estimatedTimeRemainingForState = new DoubleYoVariable("icpPlannerEstiTimeRemaining", registry);
   
   private final DoubleYoVariable initialTime = new DoubleYoVariable("icpPlannerInitialTime", registry);
   private final BooleanYoVariable comeToStop = new BooleanYoVariable("icpPlannerComeToStop", registry);
   private final BooleanYoVariable atAStop = new BooleanYoVariable("icpPlannerAtAStop", registry);

   private final BooleanYoVariable isInitialTransfer = new BooleanYoVariable("icpPlannerIsInitialTransfer", registry);
   
   private final DoubleYoVariable singleSupportDuration = new DoubleYoVariable("icpPlannerSingleSupportDuration", registry);
   private final DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("icpPlannerDoubleSupportDuration", registry);
   private final DoubleYoVariable doubleSupportInitialTransferDuration = new DoubleYoVariable("icpPlannerDoubleSupportInitialTransferDuration", registry);
   
   private final DoubleYoVariable omega0 = new DoubleYoVariable("icpPlannerOmega0", registry);

   private final YoGraphicPosition[] icpCornerPointsViz;

   //TODO: Finish YoVariablizing these to make rewindable and visualizable.

   private Point3d constantCenterOfPressure;
   private Point3d upcomingCornerPoint;

   private Point3d singleSupportStartICP;
   private final Point3d doubleSupportStartICP = new Point3d(), doubleSupportEndICP = new Point3d();
   private final Vector3d doubleSupportStartICPVelocity = new Vector3d(), doubleSupportEndICPVelocity = new Vector3d();

   private final DenseMatrix64F doubleSupportParameterMatrix = new DenseMatrix64F(3, 4);

   private boolean VISUALIZE = true;

   public SmoothICPComputer(double doubleSupportFirstStepFraction, int maxNumberOfConsideredFootsteps, YoVariableRegistry parentRegistry,
                            YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      hasBeenInitialized.set(false);
      
      if (yoGraphicsListRegistry == null)
      {
         VISUALIZE = false;
      }

      parentRegistry.addChild(registry);

      this.atAStop.set(true);
      
      this.doubleSupportFirstStepFraction = doubleSupportFirstStepFraction;
      this.maxNumberOfConsideredFootsteps = maxNumberOfConsideredFootsteps;

      for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
      {
         String constantCoPName = "constantCoP" + i;
         YoFramePoint yoFramePoint = new YoFramePoint(constantCoPName, ReferenceFrame.getWorldFrame(), registry);
         constantCentersOfPressure.add(yoFramePoint);

         if (VISUALIZE)
         {
            YoGraphicPosition dynamicGraphicPosition = new YoGraphicPosition(constantCoPName, yoFramePoint, 0.005, YoAppearance.Red());
            yoGraphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), dynamicGraphicPosition);
            yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), dynamicGraphicPosition.createArtifact());
         }
      }

      if (VISUALIZE)
      {
         icpCornerPointsViz = new YoGraphicPosition[maxNumberOfConsideredFootsteps - 1];

         for (int i = 0; i < maxNumberOfConsideredFootsteps - 1; i++)
         {
            icpCornerPointsViz[i] = new YoGraphicPosition("cornerPoint" + i, "", registry, 0.01, YoAppearance.Green());
            yoGraphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), icpCornerPointsViz[i]);
            yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), icpCornerPointsViz[i].createArtifact());
         }
      }
      else
      {
         icpCornerPointsViz = null;
      }
   }

   public void initializeSingleSupport(ArrayList<FramePoint> footLocationList, double singleSupportDuration, double doubleSupportDuration, double omega0,
           double initialTime, boolean stopIfReachedEnd)
   {
      this.omega0.set(3.0); //omega0);
      isInitialTransfer.set(false);
      comeToStop.set(false);
      atAStop.set(false);
      
      computeConstantCentersOfPressure(constantCentersOfPressure, footLocationList, maxNumberOfConsideredFootsteps, isInitialTransfer.getBooleanValue(), stopIfReachedEnd);

      this.doubleSupportDuration.set(doubleSupportDuration);
      this.singleSupportDuration.set(singleSupportDuration);
      this.doubleSupportInitialTransferDuration.set(Double.NaN);
      
      this.isDoubleSupport.set(false);

      this.initialTime.set(initialTime);

      int numberOfCornerPoints = maxNumberOfConsideredFootsteps - 1;
      double steppingDuration = singleSupportDuration + doubleSupportDuration;

      ArrayList<Point3d> constantCentersOfPressurePoint3ds = convertToListOfPoint3ds(constantCentersOfPressure);

      Point3d[] icpCornerPoints = NewDoubleSupportICPComputer.computeICPCornerPoints(numberOfCornerPoints, constantCentersOfPressurePoint3ds, steppingDuration,
                                     this.omega0.getDoubleValue());
      upcomingCornerPoint = icpCornerPoints[1];

      visualizeICPCornerPoints(icpCornerPoints, icpCornerPointsViz);

      Point3d cornerPoint0 = icpCornerPoints[0];
      Point3d supportFoot = constantCentersOfPressure.get(0).getPoint3dCopy();

      
      singleSupportStartICP = NewDoubleSupportICPComputer.computeSingleSupportStartICP(supportFoot, cornerPoint0, doubleSupportDuration,
              doubleSupportFirstStepFraction, this.omega0.getDoubleValue());
   }

   private ArrayList<Point3d> convertToListOfPoint3ds(ArrayList<YoFramePoint> yoFramePoints)
   {
      ArrayList<Point3d> ret = new ArrayList<Point3d>(yoFramePoints.size());

      for (YoFramePoint yoFramePoint : yoFramePoints)
      {
         ret.add(yoFramePoint.getPoint3dCopy());
      }

      return ret;
   }

   public List<YoFramePoint> getConstantCentersOfPressure()
   {
      return constantCentersOfPressure;
   }

   private static void computeConstantCentersOfPressure(ArrayList<YoFramePoint> constantCentersOfPressureToModify, ArrayList<FramePoint> footLocations,
           int maxNumberOfConsideredFootsteps, boolean isInitialTransfer, boolean stopIfReachedEnd)
   {
      boolean putFirstCenterOfPressureInMiddle = isInitialTransfer;
//      boolean willReachTheEnd = footLocations.size() <= maxNumberOfConsideredFootsteps;
//      boolean putTheLastNCentersOfPressureInMiddle = stopIfReachedEnd && willReachTheEnd;

      int numberInFootlist = footLocations.size();
      Point3d positionToHoldAt = new Point3d();

      for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
      {
         YoFramePoint centerOfPressureLocation = constantCentersOfPressureToModify.get(i);

         if (i == 0)
         {
            if (putFirstCenterOfPressureInMiddle)
            {
               centerOfPressureLocation.set(footLocations.get(0).getPoint());
               centerOfPressureLocation.add(footLocations.get(1).getPoint());
               centerOfPressureLocation.scale(0.5);
            }
            else
            {
               centerOfPressureLocation.set(footLocations.get(0).getPoint());
            }
         }

         else if (i < numberInFootlist - 1)
         {
            centerOfPressureLocation.set(footLocations.get(i).getPoint());
         }

         else
         {
            if (i == numberInFootlist - 1)
            {
//               if (putTheLastNCentersOfPressureInMiddle)
//               {
                  positionToHoldAt.set(footLocations.get(i).getPoint());
                  positionToHoldAt.add(footLocations.get(i - 1).getPoint());
                  positionToHoldAt.scale(0.5);
//               }
//               else
//               {
//                  positionToHoldAt.set(footLocations.get(i).getPoint());
//               }
            }

            centerOfPressureLocation.set(positionToHoldAt);
         }
      }
   }

   public void initializeDoubleSupportInitialTransfer(ArrayList<FramePoint> footLocationList, Point3d initialICPPosition, double singleSupportDuration,
           double doubleSupportDuration, double doubleSupportInitialTransferDuration, double omega0, double initialTime, boolean stopIfReachedEnd)
   {
      this.omega0.set(3.0); //omega0);

      isInitialTransfer.set(true);
      comeToStop.set(footLocationList.size() < 3);

      computeConstantCentersOfPressure(constantCentersOfPressure, footLocationList, maxNumberOfConsideredFootsteps, isInitialTransfer.getBooleanValue(), stopIfReachedEnd);

      this.doubleSupportDuration.set(doubleSupportDuration);
      this.singleSupportDuration.set(singleSupportDuration);
      this.doubleSupportInitialTransferDuration.set(doubleSupportInitialTransferDuration);
      
      this.isDoubleSupport.set(true);
      this.initialTime.set(initialTime);

      int numberOfCornerPoints = maxNumberOfConsideredFootsteps - 1;
      double steppingDuration = singleSupportDuration + doubleSupportDuration;

      ArrayList<Point3d> constantCentersOfPressurePoint3ds = convertToListOfPoint3ds(constantCentersOfPressure);

      Point3d[] icpCornerPoints = NewDoubleSupportICPComputer.computeICPCornerPoints(numberOfCornerPoints, constantCentersOfPressurePoint3ds, steppingDuration,
                                     this.omega0.getDoubleValue());
      upcomingCornerPoint = icpCornerPoints[1];

      visualizeICPCornerPoints(icpCornerPoints, icpCornerPointsViz);

      Point3d cornerPoint1 = icpCornerPoints[1];
      YoFramePoint transferToFoot = constantCentersOfPressure.get(1);

      doubleSupportStartICP.set(initialICPPosition);
      doubleSupportStartICPVelocity.set(0.0, 0.0, 0.0);

      NewDoubleSupportICPComputer.computeSingleSupportStartICPAndVelocity(doubleSupportEndICP, doubleSupportEndICPVelocity, transferToFoot.getPoint3dCopy(),
              cornerPoint1, doubleSupportDuration, doubleSupportFirstStepFraction, this.omega0.getDoubleValue());

      DoubleSupportICPComputer.computeThirdOrderPolynomialParameterMatrix(doubleSupportParameterMatrix, doubleSupportInitialTransferDuration,
              doubleSupportStartICP, doubleSupportStartICPVelocity, doubleSupportEndICP, doubleSupportEndICPVelocity);
   }


   private final Point3d icpPostionTempOne = new Point3d();
   private final Vector3d icpVelocityTempOne = new Vector3d();
   private final Point3d ecmpTempOne = new Point3d();
   
   public void initializeDoubleSupport(ArrayList<FramePoint> footLocationList, double singleSupportDuration, double doubleSupportDuration, double omega0,
           double initialTime, boolean stopIfReachedEnd)
   {
      if (atAStop.getBooleanValue())
      {
         double doubleSupportInitialTransferDuration = 1.0;
         
         if (hasBeenInitialized.getBooleanValue())
         {
            this.getICPPositionAndVelocity(icpPostionTempOne, icpVelocityTempOne, ecmpTempOne, initialTime);
         }
         else
         {
            footLocationList.get(0).get(icpPostionTempOne);
            Point3d tempPoint = new Point3d();
            footLocationList.get(1).get(tempPoint);
            icpPostionTempOne.add(tempPoint);
            icpPostionTempOne.scale(0.5);

            hasBeenInitialized.set(true);
         }
         
         initializeDoubleSupportInitialTransfer(footLocationList, icpPostionTempOne, singleSupportDuration, doubleSupportDuration, doubleSupportInitialTransferDuration, this.omega0.getDoubleValue(), initialTime, stopIfReachedEnd);
      }
      else
      {
         initializeDoubleSupportLocal(footLocationList, singleSupportDuration, doubleSupportDuration, this.omega0.getDoubleValue(),
               initialTime, stopIfReachedEnd);
      }
   }

   private void initializeDoubleSupportLocal(ArrayList<FramePoint> footLocationList, double singleSupportDuration, double doubleSupportDuration, double omega0,
         double initialTime, boolean stopIfReachedEnd)
   {
      this.omega0.set(3.0); //omega0);

      isInitialTransfer.set(false);
      comeToStop.set(footLocationList.size() < 3);
      atAStop.set(footLocationList.size() < 3);

      computeConstantCentersOfPressure(constantCentersOfPressure, footLocationList, maxNumberOfConsideredFootsteps, isInitialTransfer.getBooleanValue(), stopIfReachedEnd);


      this.doubleSupportDuration.set(doubleSupportDuration);
      this.singleSupportDuration.set(singleSupportDuration);
      this.doubleSupportInitialTransferDuration.set(Double.NaN);

      this.isDoubleSupport.set(true);

      this.initialTime.set(initialTime);

      int numberOfCornerPoints = maxNumberOfConsideredFootsteps - 1;
      double steppingDuration = singleSupportDuration + doubleSupportDuration;

      ArrayList<Point3d> constantCentersOfPressurePoint3ds = convertToListOfPoint3ds(constantCentersOfPressure);

      Point3d[] icpCornerPoints = NewDoubleSupportICPComputer.computeICPCornerPoints(numberOfCornerPoints, constantCentersOfPressurePoint3ds, steppingDuration,
            this.omega0.getDoubleValue());
      upcomingCornerPoint = icpCornerPoints[1];

      visualizeICPCornerPoints(icpCornerPoints, icpCornerPointsViz);

      Point3d cornerPoint0 = icpCornerPoints[0];
      Point3d cornerPoint1 = icpCornerPoints[1];
      Point3d transferFromFoot = constantCentersOfPressure.get(0).getPoint3dCopy();
      Point3d transferToFoot = constantCentersOfPressure.get(1).getPoint3dCopy();

      NewDoubleSupportICPComputer.computeSingleSupportEndICPAndVelocity(doubleSupportStartICP, doubleSupportStartICPVelocity, transferFromFoot, cornerPoint0,
              doubleSupportDuration, doubleSupportFirstStepFraction, singleSupportDuration, this.omega0.getDoubleValue());
      NewDoubleSupportICPComputer.computeSingleSupportStartICPAndVelocity(doubleSupportEndICP, doubleSupportEndICPVelocity, transferToFoot, cornerPoint1,
              doubleSupportDuration, doubleSupportFirstStepFraction, this.omega0.getDoubleValue());

      DoubleSupportICPComputer.computeThirdOrderPolynomialParameterMatrix(doubleSupportParameterMatrix, doubleSupportDuration, doubleSupportStartICP,
              doubleSupportStartICPVelocity, doubleSupportEndICP, doubleSupportEndICPVelocity);

   }


   private void visualizeICPCornerPoints(Point3d[] icpCornerPoints, YoGraphicPosition[] icpCornerPointsViz2)
   {
      if (VISUALIZE)
      {
         for (int i = 0; i < icpCornerPoints.length; i++)
         {
            icpCornerPointsViz[i].setPosition(icpCornerPoints[i]);
         }
      }

   }

   public Point3d getDoubleSupportStartICP()
   {
      return doubleSupportStartICP;
   }

   public Point3d getDoubleSupportEndICP()
   {
      return doubleSupportEndICP;
   }


   public Vector3d getDoubleSupportStartICPVelocity()
   {
      return doubleSupportStartICPVelocity;
   }

   public Vector3d getDoubleSupportEndICPVelocity()
   {
      return doubleSupportEndICPVelocity;
   }

   public void getICPPositionAndVelocity(Point3d icpPostionToPack, Vector3d icpVelocityToPack, Point3d ecmpToPack, 
         double time)
   {
      computeTimeInStateAndEstimatedTimeRemaining(time);

      if (isDoubleSupport.getBooleanValue())
         getICPPositionAndVelocityDoubleSupport(icpPostionToPack, icpVelocityToPack, ecmpToPack, timeInState.getDoubleValue());
      else
         getICPPositionAndVelocitySingleSupport(icpPostionToPack, icpVelocityToPack, ecmpToPack, timeInState.getDoubleValue());
   }

   private void getICPPositionAndVelocitySingleSupport(Point3d icpPositionToPack, Vector3d icpVelocityToPack, Point3d ecmpToPack, double timeInState)
   {
      constantCenterOfPressure = constantCentersOfPressure.get(0).getPoint3dCopy();
      NewDoubleSupportICPComputer.computeSingleSupportICPPositionAndVelocity(icpPositionToPack, icpVelocityToPack, constantCenterOfPressure,
              singleSupportStartICP, omega0.getDoubleValue(), timeInState);

      ecmpToPack.set(constantCenterOfPressure);
   }
   
   public double getTimeInState(double time)
   {
      computeTimeInStateAndEstimatedTimeRemaining(time);

      return timeInState.getDoubleValue(); 
   }

   private void getICPPositionAndVelocityDoubleSupport(Point3d icpPostionToPack, Vector3d icpVelocityToPack, Point3d ecmpToPack, double timeInState)
   {
      if (comeToStop.getBooleanValue() && (timeInState > doubleSupportDuration.getDoubleValue()))
      {
         icpPostionToPack.set(doubleSupportEndICP);
         icpVelocityToPack.set(0.0, 0.0, 0.0);
         ecmpToPack.set(icpPostionToPack);
      }

      else
      {
         double timePow3 = Math.pow(timeInState, 3.0);
         double timePow2 = Math.pow(timeInState, 2.0);
         Vector3d tempVector = new Vector3d();

         DenseMatrix64F dcmPositionTimeVector = new DenseMatrix64F(4, 1, true, timePow3, timePow2, timeInState, 1);
         DenseMatrix64F dcmVelocityTimeVector = new DenseMatrix64F(4, 1, true, 3 * timePow2, 2 * timeInState, 1, 0);

         multiplyMatricesAndPutInPoint3d(icpPostionToPack, doubleSupportParameterMatrix, dcmPositionTimeVector);
         multiplyMatricesAndPutInPoint3d(icpVelocityToPack, doubleSupportParameterMatrix, dcmVelocityTimeVector);
         tempVector.set(icpVelocityToPack);
         tempVector.scale(-1.0 / omega0.getDoubleValue());
         ecmpToPack.set(icpPostionToPack);
         ecmpToPack.add(tempVector);
      }
   }

   private void multiplyMatricesAndPutInPoint3d(Tuple3d tuple3dToPack, DenseMatrix64F matrixOne, DenseMatrix64F matrixTwo)
   {
      DenseMatrix64F tempMatrix = new DenseMatrix64F(3, 1);
      CommonOps.mult(matrixOne, matrixTwo, tempMatrix);

      if (tempMatrix.getNumCols() != 1)
         throw new RuntimeException("tempMatrix.getNumCols() != 1");
      if (tempMatrix.getNumRows() != 3)
         throw new RuntimeException("tempMatrix.getNumRows() != 3");

      tuple3dToPack.setX(tempMatrix.get(0, 0));
      tuple3dToPack.setY(tempMatrix.get(1, 0));
      tuple3dToPack.setZ(tempMatrix.get(2, 0));
   }
  

   private final ArrayList<FramePoint> footLocationList = new ArrayList<FramePoint>();

   public void initializeSingleSupport(TransferToAndNextFootstepsData transferToAndNextFootstepsData, double initialTime)
   {
      footLocationList.clear();
      ArrayList<ReferenceFrame> dummyReferenceFrameList = new ArrayList<ReferenceFrame>(); 

      transferToAndNextFootstepsData.getFootLocationList(footLocationList, dummyReferenceFrameList, icpForwardFromCenter.getDoubleValue(), icpInFromCenter.getDoubleValue());
      double singleSupportDuration = transferToAndNextFootstepsData.getSingleSupportDuration();
      double doubleSupportDuration = transferToAndNextFootstepsData.getDoubleSupportDuration();
      double omega0 = transferToAndNextFootstepsData.getW0();
      boolean stopIfReachedEnd = transferToAndNextFootstepsData.getStopIfReachedEnd();

      initializeSingleSupport(footLocationList, singleSupportDuration, doubleSupportDuration, omega0, initialTime, stopIfReachedEnd);
   }
   
   public void reInitializeSingleSupport(TransferToAndNextFootstepsData transferToAndNextFootstepsData, double currentTime)
   {
      footLocationList.clear();
      ArrayList<ReferenceFrame> dummyReferenceFrameList = new ArrayList<ReferenceFrame>(); 

      transferToAndNextFootstepsData.getFootLocationList(footLocationList, dummyReferenceFrameList, icpForwardFromCenter.getDoubleValue(), icpInFromCenter.getDoubleValue());
      double singleSupportDuration = transferToAndNextFootstepsData.getSingleSupportDuration();
      double doubleSupportDuration = transferToAndNextFootstepsData.getDoubleSupportDuration();
      double omega0 = transferToAndNextFootstepsData.getW0();
      boolean stopIfReachedEnd = transferToAndNextFootstepsData.getStopIfReachedEnd();

      initializeSingleSupport(footLocationList, singleSupportDuration, doubleSupportDuration, omega0, initialTime.getDoubleValue(), stopIfReachedEnd);
   }
   
   public void initializeDoubleSupportInitialTransfer(TransferToAndNextFootstepsData transferToAndNextFootstepsData, Point3d initialICPPosition,
           double initialTime)
   {
      footLocationList.clear();
      ArrayList<ReferenceFrame> dummyReferenceFrameList = new ArrayList<ReferenceFrame>(); 

      transferToAndNextFootstepsData.getFootLocationList(footLocationList, dummyReferenceFrameList, icpForwardFromCenter.getDoubleValue(), icpInFromCenter.getDoubleValue());
      double singleSupportDuration = transferToAndNextFootstepsData.getSingleSupportDuration();
      double doubleSupportDuration = transferToAndNextFootstepsData.getDoubleSupportDuration();
      double doubleSupportInitialTransferDuration = transferToAndNextFootstepsData.getDoubleSupportInitialTransferDuration();
      double omega0 = transferToAndNextFootstepsData.getW0();
      boolean stopIfReachedEnd = transferToAndNextFootstepsData.getStopIfReachedEnd();

      initializeDoubleSupportInitialTransfer(footLocationList, initialICPPosition, singleSupportDuration, doubleSupportDuration,
              doubleSupportInitialTransferDuration, omega0, initialTime, stopIfReachedEnd);
   }

   public void initializeDoubleSupport(TransferToAndNextFootstepsData transferToAndNextFootstepsData, double initialTime)
   {
      footLocationList.clear();
      ArrayList<ReferenceFrame> dummyReferenceFrameList = new ArrayList<ReferenceFrame>(); 

      transferToAndNextFootstepsData.getFootLocationList(footLocationList, dummyReferenceFrameList, icpForwardFromCenter.getDoubleValue(), icpInFromCenter.getDoubleValue());
      double singleSupportDuration = transferToAndNextFootstepsData.getSingleSupportDuration();
      double doubleSupportDuration = transferToAndNextFootstepsData.getDoubleSupportDuration();
      double omega0 = transferToAndNextFootstepsData.getW0();
      boolean stopIfReachedEnd = transferToAndNextFootstepsData.getStopIfReachedEnd();

      initializeDoubleSupport(footLocationList, singleSupportDuration, doubleSupportDuration, omega0, initialTime, stopIfReachedEnd);
   }

   private void computeTimeInStateAndEstimatedTimeRemaining(double time)
   {
      timeInState.set(time - initialTime.getDoubleValue());

      if (isDoubleSupport.getBooleanValue())
      {
         if (isInitialTransfer.getBooleanValue())
         {
            estimatedTimeRemainingForState.set(doubleSupportInitialTransferDuration.getDoubleValue() - timeInState.getDoubleValue());
         }

         else 
         {
            estimatedTimeRemainingForState.set(doubleSupportDuration.getDoubleValue()  - timeInState.getDoubleValue());
         }
      }
      else estimatedTimeRemainingForState.set(singleSupportDuration.getDoubleValue() - timeInState.getDoubleValue());
   }
   
   public boolean isDone(double time)
   {
      computeTimeInStateAndEstimatedTimeRemaining(time);

      return (estimatedTimeRemainingForState.getDoubleValue() <= -1e-4);
   }

   public Point3d getUpcomingCornerPoint()
   {
      return upcomingCornerPoint;
   } 
   
   public double getEstimatedTimeRemainingForState(double time)
   {
      return estimatedTimeRemainingForState.getDoubleValue();
   }


   public boolean isPerformingICPDoubleSupport()
   {
       return isDoubleSupport.getBooleanValue();
   }
   
   
   public void reset(double time)
   {
      boolean stopIfReachedEnd = true;
      doubleSupportInitialTransferDuration.set(0.6);
      atAStop.set(true);
      
      footLocationList.clear();
      footLocationList.add(constantCentersOfPressure.get(0).getFramePointCopy());
      footLocationList.add(constantCentersOfPressure.get(1).getFramePointCopy());
      
      double doubleSupportInitialTransferDuration = 0.4;
      
      Point3d initialICPPosition = new Point3d();
      Vector3d initialICPVelocity = new Vector3d();
      Point3d InitialECMPPosition = new Point3d();
      
      getICPPositionAndVelocity(initialICPPosition, initialICPVelocity, InitialECMPPosition, time);
      
      initializeDoubleSupportInitialTransfer(footLocationList, initialICPPosition, 
            singleSupportDuration.getDoubleValue(), doubleSupportDuration.getDoubleValue(), 
            doubleSupportInitialTransferDuration, omega0.getDoubleValue(), time, stopIfReachedEnd);
      
   
   }
   
   public void setICPInFromCenter(double icpInFromCenter)
   {
      this.icpInFromCenter.set(icpInFromCenter);
   }
}
