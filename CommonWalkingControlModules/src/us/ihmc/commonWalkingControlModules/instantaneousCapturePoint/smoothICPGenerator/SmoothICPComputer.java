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
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

public class SmoothICPComputer
{
   private final int maxNumberOfConsideredFootsteps;
   private final double doubleSupportFirstStepFraction;

   private final YoVariableRegistry registry;

   private final ArrayList<YoFramePoint> constantCentersOfPressure = new ArrayList<YoFramePoint>();

   private boolean isDoubleSupport;
   private double initialTime;
   private boolean comeToStop;
   private boolean atAStop = true;

   private boolean isInitialTransfer;
   
   private double singleSupportDuration;
   private double doubleSupportDuration;
   private double doubleSupportInitialTransferDuration;
   
   private double omega0;

   private final NewDoubleSupportICPComputer newDoubleSupportICPComputer;
   private final DynamicGraphicPosition[] icpCornerPointsViz;

   private Point3d constantCenterOfPressure;
   private Point3d upcomingCornerPoint;

   private Point3d singleSupportStartICP;
   private final Point3d doubleSupportStartICP = new Point3d(), doubleSupportEndICP = new Point3d();
   private final Vector3d doubleSupportStartICPVelocity = new Vector3d(), doubleSupportEndICPVelocity = new Vector3d();

   private final DenseMatrix64F doubleSupportParameterMatrix = new DenseMatrix64F(3, 4);

   private boolean VISUALIZE = true;

   public SmoothICPComputer(double doubleSupportFirstStepFraction, int maxNumberOfConsideredFootsteps, YoVariableRegistry parentRegistry,
                            DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      if (dynamicGraphicObjectsListRegistry == null)
      {
         VISUALIZE = false;
      }

      registry = new YoVariableRegistry(getClass().getSimpleName());
      parentRegistry.addChild(registry);

      this.doubleSupportFirstStepFraction = doubleSupportFirstStepFraction;
      this.maxNumberOfConsideredFootsteps = maxNumberOfConsideredFootsteps;

      for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
      {
         String constantCoPName = "constantCoP" + i;
         YoFramePoint yoFramePoint = new YoFramePoint(constantCoPName, ReferenceFrame.getWorldFrame(), registry);
         constantCentersOfPressure.add(yoFramePoint);

         if (VISUALIZE)
         {
            DynamicGraphicPosition dynamicGraphicPosition = new DynamicGraphicPosition(constantCoPName, yoFramePoint, 0.005, YoAppearance.Red());
            dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(getClass().getSimpleName(), dynamicGraphicPosition);
            dynamicGraphicObjectsListRegistry.registerArtifact(getClass().getSimpleName(), dynamicGraphicPosition.createArtifact());
         }
      }

      newDoubleSupportICPComputer = new NewDoubleSupportICPComputer();

      if (VISUALIZE)
      {
         icpCornerPointsViz = new DynamicGraphicPosition[maxNumberOfConsideredFootsteps - 1];

         for (int i = 0; i < maxNumberOfConsideredFootsteps - 1; i++)
         {
            icpCornerPointsViz[i] = new DynamicGraphicPosition("cornerPoint" + i, "", registry, 0.01, YoAppearance.Green());
            dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(getClass().getSimpleName(), icpCornerPointsViz[i]);
            dynamicGraphicObjectsListRegistry.registerArtifact(getClass().getSimpleName(), icpCornerPointsViz[i].createArtifact());
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
      this.omega0 = omega0;
      isInitialTransfer = false;
      comeToStop = false;
      atAStop = false;
      
      computeConstantCentersOfPressure(constantCentersOfPressure, footLocationList, maxNumberOfConsideredFootsteps, isInitialTransfer, stopIfReachedEnd);

      this.doubleSupportDuration = doubleSupportDuration;
      this.singleSupportDuration = singleSupportDuration;
      this.doubleSupportInitialTransferDuration = Double.NaN;
      
      this.isDoubleSupport = false;

      this.initialTime = initialTime;

      int numberOfCornerPoints = maxNumberOfConsideredFootsteps - 1;
      double steppingDuration = singleSupportDuration + doubleSupportDuration;

      ArrayList<Point3d> constantCentersOfPressurePoint3ds = convertToListOfPoint3ds(constantCentersOfPressure);

      Point3d[] icpCornerPoints = newDoubleSupportICPComputer.computeICPCornerPoints(numberOfCornerPoints, constantCentersOfPressurePoint3ds, steppingDuration,
                                     omega0);
      upcomingCornerPoint = icpCornerPoints[1];

      visualizeICPCornerPoints(icpCornerPoints, icpCornerPointsViz);

      Point3d cornerPoint0 = icpCornerPoints[0];
      Point3d supportFoot = constantCentersOfPressure.get(0).getPoint3dCopy();

      
      singleSupportStartICP = newDoubleSupportICPComputer.computeSingleSupportStartICP(supportFoot, cornerPoint0, doubleSupportDuration,
              doubleSupportFirstStepFraction, omega0);
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
      boolean willReachTheEnd = footLocations.size() <= maxNumberOfConsideredFootsteps;
      boolean putTheLastNCentersOfPressureInMiddle = stopIfReachedEnd && willReachTheEnd;

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
               if (putTheLastNCentersOfPressureInMiddle)
               {
                  positionToHoldAt.set(footLocations.get(i).getPoint());
                  positionToHoldAt.add(footLocations.get(i - 1).getPoint());
                  positionToHoldAt.scale(0.5);
               }
               else
               {
                  positionToHoldAt.set(footLocations.get(i).getPoint());
               }
            }

            centerOfPressureLocation.set(positionToHoldAt);
         }
      }
   }

   public void initializeDoubleSupportInitialTransfer(ArrayList<FramePoint> footLocationList, Point3d initialICPPosition, double singleSupportDuration,
           double doubleSupportDuration, double doubleSupportInitialTransferDuration, double omega0, double initialTime, boolean stopIfReachedEnd)
   {
      this.omega0 = omega0;

      isInitialTransfer = true;
      comeToStop = footLocationList.size() < 3;

      computeConstantCentersOfPressure(constantCentersOfPressure, footLocationList, maxNumberOfConsideredFootsteps, isInitialTransfer, stopIfReachedEnd);

      this.doubleSupportDuration = doubleSupportDuration;
      this.singleSupportDuration = singleSupportDuration;
      this.doubleSupportInitialTransferDuration = doubleSupportInitialTransferDuration;
      
      this.isDoubleSupport = true;
      this.initialTime = initialTime;

      int numberOfCornerPoints = maxNumberOfConsideredFootsteps - 1;
      double steppingDuration = singleSupportDuration + doubleSupportDuration;

      ArrayList<Point3d> constantCentersOfPressurePoint3ds = convertToListOfPoint3ds(constantCentersOfPressure);

      Point3d[] icpCornerPoints = newDoubleSupportICPComputer.computeICPCornerPoints(numberOfCornerPoints, constantCentersOfPressurePoint3ds, steppingDuration,
                                     omega0);
      upcomingCornerPoint = icpCornerPoints[1];

      visualizeICPCornerPoints(icpCornerPoints, icpCornerPointsViz);

      Point3d cornerPoint1 = icpCornerPoints[1];
      YoFramePoint transferToFoot = constantCentersOfPressure.get(1);

      doubleSupportStartICP.set(initialICPPosition);
      doubleSupportStartICPVelocity.set(0.0, 0.0, 0.0);

      newDoubleSupportICPComputer.computeSingleSupportStartICPAndVelocity(doubleSupportEndICP, doubleSupportEndICPVelocity, transferToFoot.getPoint3dCopy(),
              cornerPoint1, doubleSupportDuration, doubleSupportFirstStepFraction, omega0);

      DoubleSupportICPComputer.computeThirdOrderPolynomialParameterMatrix(doubleSupportParameterMatrix, doubleSupportInitialTransferDuration,
              doubleSupportStartICP, doubleSupportStartICPVelocity, doubleSupportEndICP, doubleSupportEndICPVelocity);
   }


   private final Point3d icpPostionTempOne = new Point3d();
//   private final Vector3d icpVelocityTempOne = new Vector3d();
//   private final Point3d ecmpTempOne = new Point3d();
   
   public void initializeDoubleSupport(ArrayList<FramePoint> footLocationList, double singleSupportDuration, double doubleSupportDuration, double omega0,
           double initialTime, boolean stopIfReachedEnd)
   {
      if (atAStop)
      {
         double doubleSupportInitialTransferDuration = 1.0;
//         this.getICPPositionAndVelocity(icpPostionTempOne, icpVelocityTempOne, ecmpTempOne, initialTime);
         
         icpPostionTempOne.set(footLocationList.get(0).getPointCopy());
         icpPostionTempOne.add(footLocationList.get(1).getPointCopy());
         icpPostionTempOne.scale(0.5);
         
         initializeDoubleSupportInitialTransfer(footLocationList, icpPostionTempOne, singleSupportDuration, doubleSupportDuration, doubleSupportInitialTransferDuration, omega0, initialTime, stopIfReachedEnd);
      }
      else
      {
         initializeDoubleSupportLocal(footLocationList, singleSupportDuration, doubleSupportDuration, omega0,
               initialTime, stopIfReachedEnd);
      }
   }

   private void initializeDoubleSupportLocal(ArrayList<FramePoint> footLocationList, double singleSupportDuration, double doubleSupportDuration, double omega0,
         double initialTime, boolean stopIfReachedEnd)
   {
      this.omega0 = omega0;

      isInitialTransfer = false;
      comeToStop = footLocationList.size() < 3;
      atAStop = footLocationList.size() < 3;

      computeConstantCentersOfPressure(constantCentersOfPressure, footLocationList, maxNumberOfConsideredFootsteps, isInitialTransfer, stopIfReachedEnd);


      this.doubleSupportDuration = doubleSupportDuration;
      this.singleSupportDuration = singleSupportDuration;
      this.doubleSupportInitialTransferDuration = Double.NaN;

      this.isDoubleSupport = true;

      this.initialTime = initialTime;

      int numberOfCornerPoints = maxNumberOfConsideredFootsteps - 1;
      double steppingDuration = singleSupportDuration + doubleSupportDuration;

      ArrayList<Point3d> constantCentersOfPressurePoint3ds = convertToListOfPoint3ds(constantCentersOfPressure);

      Point3d[] icpCornerPoints = newDoubleSupportICPComputer.computeICPCornerPoints(numberOfCornerPoints, constantCentersOfPressurePoint3ds, steppingDuration,
            omega0);
      upcomingCornerPoint = icpCornerPoints[1];

      visualizeICPCornerPoints(icpCornerPoints, icpCornerPointsViz);

      Point3d cornerPoint0 = icpCornerPoints[0];
      Point3d cornerPoint1 = icpCornerPoints[1];
      Point3d transferFromFoot = constantCentersOfPressure.get(0).getPoint3dCopy();
      Point3d transferToFoot = constantCentersOfPressure.get(1).getPoint3dCopy();

      newDoubleSupportICPComputer.computeSingleSupportEndICPAndVelocity(doubleSupportStartICP, doubleSupportStartICPVelocity, transferFromFoot, cornerPoint0,
              doubleSupportDuration, doubleSupportFirstStepFraction, singleSupportDuration, omega0);
      newDoubleSupportICPComputer.computeSingleSupportStartICPAndVelocity(doubleSupportEndICP, doubleSupportEndICPVelocity, transferToFoot, cornerPoint1,
              doubleSupportDuration, doubleSupportFirstStepFraction, omega0);

      DoubleSupportICPComputer.computeThirdOrderPolynomialParameterMatrix(doubleSupportParameterMatrix, doubleSupportDuration, doubleSupportStartICP,
              doubleSupportStartICPVelocity, doubleSupportEndICP, doubleSupportEndICPVelocity);

   }


   private void visualizeICPCornerPoints(Point3d[] icpCornerPoints, DynamicGraphicPosition[] icpCornerPointsViz2)
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

   public void getICPPositionAndVelocity(Point3d icpPostionToPack, Vector3d icpVelocityToPack, Point3d ecmpToPack, double time)
   {
      double timeInState = time - initialTime;

      if (isDoubleSupport)
         getICPPositionAndVelocityDoubleSupport(icpPostionToPack, icpVelocityToPack, ecmpToPack, timeInState);
      else
         getICPPositionAndVelocitySingleSupport(icpPostionToPack, icpVelocityToPack, ecmpToPack, timeInState);
   }

   private void getICPPositionAndVelocitySingleSupport(Point3d icpPositionToPack, Vector3d icpVelocityToPack, Point3d ecmpToPack, double timeInState)
   {
      constantCenterOfPressure = constantCentersOfPressure.get(0).getPoint3dCopy();
      newDoubleSupportICPComputer.computeSingleSupportICPPositionAndVelocity(icpPositionToPack, icpVelocityToPack, constantCenterOfPressure,
              singleSupportStartICP, omega0, timeInState);

      ecmpToPack.set(constantCenterOfPressure);
   }

   private void getICPPositionAndVelocityDoubleSupport(Point3d icpPostionToPack, Vector3d icpVelocityToPack, Point3d ecmpToPack, double timeInState)
   {
      if (comeToStop && (timeInState > doubleSupportDuration))
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
         tempVector.scale(-1.0 / omega0);
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

      transferToAndNextFootstepsData.getFootLocationList(footLocationList);
      double singleSupportDuration = transferToAndNextFootstepsData.getSingleSupportDuration();
      double doubleSupportDuration = transferToAndNextFootstepsData.getDoubleSupportDuration();
      double omega0 = transferToAndNextFootstepsData.getW0();
      boolean stopIfReachedEnd = transferToAndNextFootstepsData.getStopIfReachedEnd();

      initializeSingleSupport(footLocationList, singleSupportDuration, doubleSupportDuration, omega0, initialTime, stopIfReachedEnd);
   }
   
   public void initializeDoubleSupportInitialTransfer(TransferToAndNextFootstepsData transferToAndNextFootstepsData, Point3d initialICPPosition,
           double initialTime)
   {
      footLocationList.clear();

      transferToAndNextFootstepsData.getFootLocationList(footLocationList);
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

      transferToAndNextFootstepsData.getFootLocationList(footLocationList);
      double singleSupportDuration = transferToAndNextFootstepsData.getSingleSupportDuration();
      double doubleSupportDuration = transferToAndNextFootstepsData.getDoubleSupportDuration();
      double omega0 = transferToAndNextFootstepsData.getW0();
      boolean stopIfReachedEnd = transferToAndNextFootstepsData.getStopIfReachedEnd();

      initializeDoubleSupport(footLocationList, singleSupportDuration, doubleSupportDuration, omega0, initialTime, stopIfReachedEnd);

   }

   public boolean isDone(double time)
   {
      double timeInState = time - initialTime;
      if (isDoubleSupport)
      {
         if (isInitialTransfer)
         {
            return timeInState > doubleSupportInitialTransferDuration;
         }

         else 
         {
            return timeInState > doubleSupportDuration;
         }
      }

      else return timeInState > singleSupportDuration;
   }

   public Point3d getUpcomingCornerPoint()
   {
      return upcomingCornerPoint;
   }
   
   
   public void reset(double time)
   {
      boolean stopIfReachedEnd = true;
      doubleSupportInitialTransferDuration = 0.6;
      atAStop = true;
      
      footLocationList.clear();
      footLocationList.add(constantCentersOfPressure.get(0).getFramePointCopy());
      footLocationList.add(constantCentersOfPressure.get(1).getFramePointCopy());
      
      double doubleSupportInitialTransferDuration = 0.4;
      
      Point3d initialICPPosition = new Point3d();
      Vector3d initialICPVelocity = new Vector3d();
      Point3d InitialECMPPosition = new Point3d();
      
      getICPPositionAndVelocity(initialICPPosition, initialICPVelocity, InitialECMPPosition, time);
      
      initializeDoubleSupportInitialTransfer(footLocationList, initialICPPosition, singleSupportDuration, doubleSupportDuration, doubleSupportInitialTransferDuration, omega0, time, stopIfReachedEnd);
      
   
   }
}
