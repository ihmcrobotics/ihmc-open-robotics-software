package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class DoubleSupportFootCenterToToeICPComputer
{
   private final int maxNumberOfConsideredFootsteps;
   private final double doubleSupportFirstStepFraction;
   private final double singleSupportToePercentage;
   private boolean doHeelToToeTransfer;
   private int numberOfCornerPoints;


   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ArrayList<YoFramePoint> constantFootCenterCentersOfPressure = new ArrayList<YoFramePoint>();
   private final ArrayList<YoFramePoint> constantToeCentersOfPressure = new ArrayList<YoFramePoint>();

   private final ArrayList<YoFramePoint> footCenterICPCornerFramePoints = new ArrayList<YoFramePoint>();
   private final ArrayList<YoFramePoint> toeICPCornerFramePoints = new ArrayList<YoFramePoint>();

   private final DoubleYoVariable icpForwardFromCenter = new DoubleYoVariable("icpForwardFromCenter", registry);

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
   
   
   
   private final Point3d desiredICPPosition = new Point3d();
   private final Vector3d desiredICPVelocity = new Vector3d();
   private final Point3d desiredECMP = new Point3d();
   
   private final YoFramePoint desiredICPPositionFramePoint = new YoFramePoint("desiredICPPosition", ReferenceFrame.getWorldFrame(),registry);
   private final YoFrameVector desiredICPVelocityFrameVector = new YoFrameVector("desiredICPVelocity", ReferenceFrame.getWorldFrame(),registry);
   private final YoFramePoint desiredECMPFramePoint = new YoFramePoint("desiredECMPPosition", ReferenceFrame.getWorldFrame(),registry);
   
   
   private final Point3d comPositionVector = new Point3d();
   private final Vector3d comVelocityVector = new Vector3d();
   
   private final YoFramePoint desiredComPositionFramePoint = new YoFramePoint("desiredComPosition", ReferenceFrame.getWorldFrame(),registry);
   private final YoFrameVector desiredComVelocityFrameVector = new YoFrameVector("desiredComVelocity", ReferenceFrame.getWorldFrame(),registry);


   
   private final DoubleYoVariable desiredICPvelAbsolute = new DoubleYoVariable("desiredICPvelAbsolute", registry);
   private final DoubleYoVariable desiredCOMvelAbsolute = new DoubleYoVariable("desiredCOMvelAbsolute", registry);

   private final DoubleYoVariable omega0 = new DoubleYoVariable("icpPlannerOmega0", registry);
   
   private final double frontalToeOffset = 0.08;


   // TODO: Finish YoVariablizing these to make rewindable and visualizable.

   private Point3d constantCenterOfPressure;
   private Point3d upcomingCornerPoint;

   private Point3d singleSupportStartICP = new Point3d();
   private Vector3d singleSupportStartICPVelocity = new Vector3d();
   private Point3d singleSupportEndICP = new Point3d();
   private Vector3d singleSupportEndICPVelocity = new Vector3d();


   private final Point3d doubleSupportEndICP = new Point3d();
   private final Vector3d doubleSupportEndICPVelocity = new Vector3d();

   private final DenseMatrix64F doubleSupportParameterMatrix = new DenseMatrix64F(3, 4);
   private final DenseMatrix64F singleSupportParameterMatrix = new DenseMatrix64F(3, 4);



   public DoubleSupportFootCenterToToeICPComputer(double doubleSupportFirstStepFraction, boolean doHeelToToeTransfer,
           int maxNumberOfConsideredFootsteps, YoVariableRegistry parentRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      this.singleSupportToePercentage = 0.5;
      this.doHeelToToeTransfer = doHeelToToeTransfer;

      hasBeenInitialized.set(false);

      parentRegistry.addChild(registry);

      this.atAStop.set(true);

      this.doubleSupportFirstStepFraction = doubleSupportFirstStepFraction;
      this.maxNumberOfConsideredFootsteps = maxNumberOfConsideredFootsteps;

      this.numberOfCornerPoints = maxNumberOfConsideredFootsteps - 1;


      for (int i = 0; i < maxNumberOfConsideredFootsteps - 1; i++)
      {
         String icpCornerPointsName = "icpCornerPoints" + i;
         YoFramePoint yoFramePoint = new YoFramePoint(icpCornerPointsName, ReferenceFrame.getWorldFrame(), registry);
         footCenterICPCornerFramePoints.add(yoFramePoint);

         if (doHeelToToeTransfer)
         {
            String toeICPCornerPointsName = "toeICPCornerPointsName" + i;
            YoFramePoint yoToeFramePoint = new YoFramePoint(toeICPCornerPointsName, ReferenceFrame.getWorldFrame(), registry);
            toeICPCornerFramePoints.add(yoToeFramePoint);
         }
      }


      for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
      {
         String constantCoPName = "constantCoP" + i;
         YoFramePoint yoFramePoint = new YoFramePoint(constantCoPName, ReferenceFrame.getWorldFrame(), registry);
         constantFootCenterCentersOfPressure.add(yoFramePoint);

         if (doHeelToToeTransfer)
         {
            String constantToeCoPName = "constantToeCoP" + i;
            YoFramePoint toeFramePoint = new YoFramePoint(constantToeCoPName, ReferenceFrame.getWorldFrame(), registry);
            constantToeCentersOfPressure.add(toeFramePoint);
         }
      }

   }

   public void initializeSingleSupport(ArrayList<FramePoint> footLocationList, double singleSupportDuration, double doubleSupportDuration, double omega0,
           double initialTime)
   {
      this.omega0.set(3.0); //omega0); // TODO: Magic number
      isInitialTransfer.set(false);
      comeToStop.set(false);
      atAStop.set(false);

      this.doubleSupportDuration.set(doubleSupportDuration);
      this.singleSupportDuration.set(singleSupportDuration);
      this.doubleSupportInitialTransferDuration.set(Double.NaN);

      this.isDoubleSupport.set(false);

      this.initialTime.set(initialTime);

      double steppingDuration = singleSupportDuration + doubleSupportDuration;


      if (doHeelToToeTransfer)
      {
         double toeToFootCenterShiftDuration = steppingDuration * singleSupportToePercentage;
         double footCenterToToeShiftDuration = steppingDuration * (1 - singleSupportToePercentage);
         double toeShiftToDoubleSupportDuration = toeToFootCenterShiftDuration - doubleSupportDuration * doubleSupportFirstStepFraction;
         double secondDoubleSupportFractionDuration = doubleSupportDuration * (1 - doubleSupportFirstStepFraction);

         NewDoubleSupportICPComputer.computeConstantCentersOfPressureAndCornerPointsForFootCenterAndToe(constantFootCenterCentersOfPressure,
                 constantToeCentersOfPressure, footCenterICPCornerFramePoints, toeICPCornerFramePoints, frontalToeOffset, footLocationList,
                 maxNumberOfConsideredFootsteps, isInitialTransfer.getBooleanValue(), this.omega0.getDoubleValue(), toeToFootCenterShiftDuration,
                 footCenterToToeShiftDuration);
         
         upcomingCornerPoint = footCenterICPCornerFramePoints.get(1).getPoint3dCopy();;


         JojosICPutilities.extrapolateDCMposAndVel(singleSupportStartICP, singleSupportStartICPVelocity,
                 constantFootCenterCentersOfPressure.get(0).getPoint3dCopy(), secondDoubleSupportFractionDuration, omega0,
                 footCenterICPCornerFramePoints.get(0).getPoint3dCopy());

         JojosICPutilities.extrapolateDCMposAndVel(singleSupportEndICP, singleSupportEndICPVelocity, constantToeCentersOfPressure.get(0).getPoint3dCopy(),
                 toeShiftToDoubleSupportDuration, omega0, toeICPCornerFramePoints.get(0).getPoint3dCopy());



         DoubleSupportICPComputer.computeThirdOrderPolynomialParameterMatrix(singleSupportParameterMatrix, singleSupportDuration, singleSupportStartICP,
                 singleSupportStartICPVelocity, singleSupportEndICP, singleSupportEndICPVelocity);

      }
      else
      {
         NewDoubleSupportICPComputer.computeConstantCentersOfPressure(constantFootCenterCentersOfPressure, footLocationList, maxNumberOfConsideredFootsteps,
                 isInitialTransfer.getBooleanValue());
         ArrayList<Point3d> constantCentersOfPressurePoint3ds = convertToListOfPoint3ds(constantFootCenterCentersOfPressure);

         Point3d[] icpCornerPoints = NewDoubleSupportICPComputer.computeICPCornerPoints(numberOfCornerPoints, constantCentersOfPressurePoint3ds,
                                        steppingDuration, this.omega0.getDoubleValue());

         for (int i = 0; i < icpCornerPoints.length; i++)
         {
            footCenterICPCornerFramePoints.get(i).set(icpCornerPoints[i]);
         }

         Point3d cornerPoint0 = icpCornerPoints[0];
         Point3d supportFoot = constantFootCenterCentersOfPressure.get(0).getPoint3dCopy();


         singleSupportStartICP = NewDoubleSupportICPComputer.computeSingleSupportStartICP(supportFoot, cornerPoint0, doubleSupportDuration,
                 doubleSupportFirstStepFraction, this.omega0.getDoubleValue());
      }
   }


   public void initializeDoubleSupportInitialTransfer(ArrayList<FramePoint> footLocationList, Point3d initialICPPosition, double singleSupportDuration,
           double doubleSupportDuration, double doubleSupportInitialTransferDuration, double omega0, double initialTime)
   {
      this.omega0.set(3.0); //omega0); // TODO: Magic number

      isInitialTransfer.set(true);
      comeToStop.set(footLocationList.size() < 3);

      this.doubleSupportDuration.set(doubleSupportDuration);
      this.singleSupportDuration.set(singleSupportDuration);
      this.doubleSupportInitialTransferDuration.set(doubleSupportInitialTransferDuration);

      this.isDoubleSupport.set(true);
      this.initialTime.set(initialTime);
      
      double steppingDuration = singleSupportDuration + doubleSupportDuration;
      double toeToFootCenterShiftDuration = steppingDuration * singleSupportToePercentage;
      double footCenterToToeShiftDuration = steppingDuration * (1 - singleSupportToePercentage);
      
      
      if (doHeelToToeTransfer)
      {
         NewDoubleSupportICPComputer.computeConstantCentersOfPressureAndCornerPointsForFootCenterAndToe(constantFootCenterCentersOfPressure,
               constantToeCentersOfPressure, footCenterICPCornerFramePoints, toeICPCornerFramePoints, frontalToeOffset, footLocationList,
               maxNumberOfConsideredFootsteps, isInitialTransfer.getBooleanValue(), this.omega0.getDoubleValue(), toeToFootCenterShiftDuration,
               footCenterToToeShiftDuration);
      }
      else
      {
         NewDoubleSupportICPComputer.computeConstantCentersOfPressure(constantFootCenterCentersOfPressure, footLocationList, maxNumberOfConsideredFootsteps,
               isInitialTransfer.getBooleanValue());
         
         int numberOfCornerPoints = maxNumberOfConsideredFootsteps - 1;
         

         ArrayList<Point3d> constantCentersOfPressurePoint3ds = convertToListOfPoint3ds(constantFootCenterCentersOfPressure);

         Point3d[] icpCornerPoints = NewDoubleSupportICPComputer.computeICPCornerPoints(numberOfCornerPoints, constantCentersOfPressurePoint3ds, steppingDuration,
                                        this.omega0.getDoubleValue());



         for (int i = 0; i < icpCornerPoints.length; i++)
         {
            footCenterICPCornerFramePoints.get(i).set(icpCornerPoints[i]);
         }
      }
      
      upcomingCornerPoint = footCenterICPCornerFramePoints.get(1).getPoint3dCopy();;

      singleSupportEndICP.set(initialICPPosition);
      singleSupportEndICPVelocity.set(0.0, 0.0, 0.0);
      
      if (doHeelToToeTransfer)
      {
         double secondDoubleSupportFractionDuration = doubleSupportDuration * (1 - doubleSupportFirstStepFraction);

         JojosICPutilities.extrapolateDCMposAndVel(doubleSupportEndICP, doubleSupportEndICPVelocity,
                 constantFootCenterCentersOfPressure.get(1).getPoint3dCopy(), secondDoubleSupportFractionDuration, omega0,
                 footCenterICPCornerFramePoints.get(1).getPoint3dCopy());

      }
      else
      {
         Point3d cornerPoint1 = footCenterICPCornerFramePoints.get(1).getPoint3dCopy();
         Point3d transferToFoot = constantFootCenterCentersOfPressure.get(1).getPoint3dCopy();
         
         NewDoubleSupportICPComputer.computeSingleSupportStartICPAndVelocity(doubleSupportEndICP, doubleSupportEndICPVelocity, transferToFoot, cornerPoint1,
                 doubleSupportDuration, doubleSupportFirstStepFraction, this.omega0.getDoubleValue());
      }

      DoubleSupportICPComputer.computeThirdOrderPolynomialParameterMatrix(doubleSupportParameterMatrix, doubleSupportInitialTransferDuration,
            singleSupportEndICP, singleSupportEndICPVelocity, doubleSupportEndICP, doubleSupportEndICPVelocity);
   }


   public void initializeDoubleSupport(ArrayList<FramePoint> footLocationList, double singleSupportDuration, double doubleSupportDuration, double omega0,
           double initialTime)
   {
      if (atAStop.getBooleanValue())
      {
         double doubleSupportInitialTransferDuration = 1.0;

         if (hasBeenInitialized.getBooleanValue())
         {
            this.computeICPPositionAndVelocity(desiredICPPosition, desiredICPVelocity, desiredECMP, initialTime);
         }
         else
         {
            desiredICPPosition.set(footLocationList.get(0).getPointCopy());
            desiredICPPosition.add(footLocationList.get(1).getPointCopy());
            desiredICPPosition.scale(0.5);

            hasBeenInitialized.set(true);
         }

         initializeDoubleSupportInitialTransfer(footLocationList, desiredICPPosition, singleSupportDuration, doubleSupportDuration,
                 doubleSupportInitialTransferDuration, this.omega0.getDoubleValue(), initialTime);
      }
      else
      {
         initializeDoubleSupportLocal(footLocationList, singleSupportDuration, doubleSupportDuration, this.omega0.getDoubleValue(), initialTime);
      }
   }

   private void initializeDoubleSupportLocal(ArrayList<FramePoint> footLocationList, double singleSupportDuration, double doubleSupportDuration, double omega0,
           double initialTime)
   {
      this.omega0.set(3.0); //omega0); // TODO: Magic number

      isInitialTransfer.set(false);
      comeToStop.set(footLocationList.size() < 3);
      atAStop.set(footLocationList.size() < 3);

      NewDoubleSupportICPComputer.computeConstantCentersOfPressure(constantFootCenterCentersOfPressure, footLocationList, maxNumberOfConsideredFootsteps,
              isInitialTransfer.getBooleanValue());


      this.doubleSupportDuration.set(doubleSupportDuration);
      this.singleSupportDuration.set(singleSupportDuration);
      this.doubleSupportInitialTransferDuration.set(Double.NaN);

      this.isDoubleSupport.set(true);

      this.initialTime.set(initialTime);

      int numberOfCornerPoints = maxNumberOfConsideredFootsteps - 1;
      double steppingDuration = singleSupportDuration + doubleSupportDuration;



      if (doHeelToToeTransfer)
      {
         double toeToFootCenterShiftDuration = steppingDuration * singleSupportToePercentage;
         double toeShiftToDoubleSupportDuration = toeToFootCenterShiftDuration - doubleSupportDuration * doubleSupportFirstStepFraction;
         double secondDoubleSupportFractionDuration = doubleSupportDuration * (1 - doubleSupportFirstStepFraction);

         JojosICPutilities.extrapolateDCMposAndVel(singleSupportEndICP, singleSupportEndICPVelocity, constantToeCentersOfPressure.get(0).getPoint3dCopy(),
                 toeShiftToDoubleSupportDuration, omega0, toeICPCornerFramePoints.get(0).getPoint3dCopy());

         JojosICPutilities.extrapolateDCMposAndVel(doubleSupportEndICP, doubleSupportEndICPVelocity,
                 constantFootCenterCentersOfPressure.get(1).getPoint3dCopy(), secondDoubleSupportFractionDuration, omega0,
                 footCenterICPCornerFramePoints.get(1).getPoint3dCopy());

      }
      else
      {
         ArrayList<Point3d> constantCentersOfPressurePoint3ds = convertToListOfPoint3ds(constantFootCenterCentersOfPressure);

         Point3d[] icpCornerPoints = NewDoubleSupportICPComputer.computeICPCornerPoints(numberOfCornerPoints, constantCentersOfPressurePoint3ds, steppingDuration,
                                        this.omega0.getDoubleValue());
         
         Point3d cornerPoint0 = icpCornerPoints[0];
         Point3d cornerPoint1 = icpCornerPoints[1];
         Point3d transferFromFoot = constantFootCenterCentersOfPressure.get(0).getPoint3dCopy();
         Point3d transferToFoot = constantFootCenterCentersOfPressure.get(1).getPoint3dCopy();

         NewDoubleSupportICPComputer.computeSingleSupportEndICPAndVelocity(singleSupportEndICP, singleSupportEndICPVelocity, transferFromFoot, cornerPoint0,
                 doubleSupportDuration, doubleSupportFirstStepFraction, singleSupportDuration, this.omega0.getDoubleValue());
         NewDoubleSupportICPComputer.computeSingleSupportStartICPAndVelocity(doubleSupportEndICP, doubleSupportEndICPVelocity, transferToFoot, cornerPoint1,
                 doubleSupportDuration, doubleSupportFirstStepFraction, this.omega0.getDoubleValue());
      }
      
      upcomingCornerPoint = footCenterICPCornerFramePoints.get(1).getPoint3dCopy();

      DoubleSupportICPComputer.computeThirdOrderPolynomialParameterMatrix(doubleSupportParameterMatrix, doubleSupportDuration, singleSupportEndICP,
              singleSupportEndICPVelocity, doubleSupportEndICP, doubleSupportEndICPVelocity);

   }



   public Point3d getDoubleSupportStartICP()
   {
      return singleSupportEndICP;
   }

   public Point3d getDoubleSupportEndICP()
   {
      return doubleSupportEndICP;
   }


   public Vector3d getDoubleSupportStartICPVelocity()
   {
      return singleSupportEndICPVelocity;
   }

   public Vector3d getDoubleSupportEndICPVelocity()
   {
      return doubleSupportEndICPVelocity;
   }

   public void computeICPPositionAndVelocity(Point3d icpPostionToPack, Vector3d icpVelocityToPack, Point3d ecmpToPack, double time)
   {
      computeTimeInStateAndEstimatedTimeRemaining(time);

      if (isDoubleSupport.getBooleanValue())
         getICPPositionAndVelocityDoubleSupport(icpPostionToPack, icpVelocityToPack, ecmpToPack, timeInState.getDoubleValue());
      else
         getICPPositionAndVelocitySingleSupport(icpPostionToPack, icpVelocityToPack, ecmpToPack, timeInState.getDoubleValue());
      

      desiredICPPosition.set(icpPostionToPack);
      desiredICPVelocity.set(icpVelocityToPack); 
      desiredECMP.set(ecmpToPack);
      
      desiredICPPositionFramePoint.set(icpPostionToPack);
      desiredICPVelocityFrameVector.set(icpVelocityToPack);
      desiredECMPFramePoint.set(ecmpToPack);
   }

   private void getICPPositionAndVelocitySingleSupport(Point3d icpPositionToPack, Vector3d icpVelocityToPack, Point3d ecmpToPack, double timeInState)
   {
      if (doHeelToToeTransfer)
      {
         double timePow3 = Math.pow(timeInState, 3.0);
         double timePow2 = Math.pow(timeInState, 2.0);
         Vector3d tempVector = new Vector3d();

         DenseMatrix64F dcmPositionTimeVector = new DenseMatrix64F(4, 1, true, timePow3, timePow2, timeInState, 1);
         DenseMatrix64F dcmVelocityTimeVector = new DenseMatrix64F(4, 1, true, 3 * timePow2, 2 * timeInState, 1, 0);

         multiplyMatricesAndPutInPoint3d(icpPositionToPack, singleSupportParameterMatrix, dcmPositionTimeVector);
         multiplyMatricesAndPutInPoint3d(icpVelocityToPack, singleSupportParameterMatrix, dcmVelocityTimeVector);
         tempVector.set(icpVelocityToPack);
         tempVector.scale(-1.0 / omega0.getDoubleValue());
         ecmpToPack.set(icpPositionToPack);
         ecmpToPack.add(tempVector);

      }
      else
      {
         constantCenterOfPressure = constantFootCenterCentersOfPressure.get(0).getPoint3dCopy();
         NewDoubleSupportICPComputer.computeSingleSupportICPPositionAndVelocity(icpPositionToPack, icpVelocityToPack, constantCenterOfPressure,
                 singleSupportStartICP, omega0.getDoubleValue(), timeInState);

         ecmpToPack.set(constantCenterOfPressure);
      }
   }

   public double getTimeInState(double time)
   {
      computeTimeInStateAndEstimatedTimeRemaining(time);

      return timeInState.getDoubleValue();
   }

   private void getICPPositionAndVelocityDoubleSupport(Point3d icpPositionToPack, Vector3d icpVelocityToPack, Point3d ecmpToPack, double timeInState)
   {
      if (comeToStop.getBooleanValue() && (timeInState > doubleSupportDuration.getDoubleValue()))
      {
         icpPositionToPack.set(doubleSupportEndICP);
         icpVelocityToPack.set(0.0, 0.0, 0.0);
         ecmpToPack.set(icpPositionToPack);
      }

      else
      {
         double timePow3 = Math.pow(timeInState, 3.0);
         double timePow2 = Math.pow(timeInState, 2.0);
         Vector3d tempVector = new Vector3d();

         DenseMatrix64F dcmPositionTimeVector = new DenseMatrix64F(4, 1, true, timePow3, timePow2, timeInState, 1);
         DenseMatrix64F dcmVelocityTimeVector = new DenseMatrix64F(4, 1, true, 3 * timePow2, 2 * timeInState, 1, 0);

         multiplyMatricesAndPutInPoint3d(icpPositionToPack, doubleSupportParameterMatrix, dcmPositionTimeVector);
         multiplyMatricesAndPutInPoint3d(icpVelocityToPack, doubleSupportParameterMatrix, dcmVelocityTimeVector);
         tempVector.set(icpVelocityToPack);
         tempVector.scale(-1.0 / omega0.getDoubleValue());
         ecmpToPack.set(icpPositionToPack);
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

      transferToAndNextFootstepsData.getFootLocationList(footLocationList, icpForwardFromCenter.getDoubleValue());
      double singleSupportDuration = transferToAndNextFootstepsData.getSingleSupportDuration();
      double doubleSupportDuration = transferToAndNextFootstepsData.getDoubleSupportDuration();
      double omega0 = transferToAndNextFootstepsData.getW0();

      initializeSingleSupport(footLocationList, singleSupportDuration, doubleSupportDuration, omega0, initialTime);
   }

   public void reInitializeSingleSupport(TransferToAndNextFootstepsData transferToAndNextFootstepsData, double currentTime)
   {
      footLocationList.clear();

      transferToAndNextFootstepsData.getFootLocationList(footLocationList, icpForwardFromCenter.getDoubleValue());
      double singleSupportDuration = transferToAndNextFootstepsData.getSingleSupportDuration();
      double doubleSupportDuration = transferToAndNextFootstepsData.getDoubleSupportDuration();
      double omega0 = transferToAndNextFootstepsData.getW0();

      initializeSingleSupport(footLocationList, singleSupportDuration, doubleSupportDuration, omega0, initialTime.getDoubleValue());
   }

   public void initializeDoubleSupportInitialTransfer(TransferToAndNextFootstepsData transferToAndNextFootstepsData, Point3d initialICPPosition,
           double initialTime)
   {
      footLocationList.clear();

      transferToAndNextFootstepsData.getFootLocationList(footLocationList, icpForwardFromCenter.getDoubleValue());
      double singleSupportDuration = transferToAndNextFootstepsData.getSingleSupportDuration();
      double doubleSupportDuration = transferToAndNextFootstepsData.getDoubleSupportDuration();
      double doubleSupportInitialTransferDuration = transferToAndNextFootstepsData.getDoubleSupportInitialTransferDuration();
      double omega0 = transferToAndNextFootstepsData.getW0();

      initializeDoubleSupportInitialTransfer(footLocationList, initialICPPosition, singleSupportDuration, doubleSupportDuration,
              doubleSupportInitialTransferDuration, omega0, initialTime);
   }

   public void initializeDoubleSupport(TransferToAndNextFootstepsData transferToAndNextFootstepsData, double initialTime)
   {
      footLocationList.clear();

      transferToAndNextFootstepsData.getFootLocationList(footLocationList, icpForwardFromCenter.getDoubleValue());
      double singleSupportDuration = transferToAndNextFootstepsData.getSingleSupportDuration();
      double doubleSupportDuration = transferToAndNextFootstepsData.getDoubleSupportDuration();
      double omega0 = transferToAndNextFootstepsData.getW0();

      initializeDoubleSupport(footLocationList, singleSupportDuration, doubleSupportDuration, omega0, initialTime);
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
            estimatedTimeRemainingForState.set(doubleSupportDuration.getDoubleValue() - timeInState.getDoubleValue());
         }
      }
      else
         estimatedTimeRemainingForState.set(singleSupportDuration.getDoubleValue() - timeInState.getDoubleValue());
   }
   
   
   public void computeDesiredICPAndCOMVelocityAbsolutes(Point3d icpVelocityToPack, double simDT)
   {
   desiredICPvelAbsolute.set(Math.sqrt(Math.pow(icpVelocityToPack.getX(), 2) + Math.pow(icpVelocityToPack.getY(), 2)
         + Math.pow(icpVelocityToPack.getZ(), 2)));
   

   JojosICPutilities.discreteIntegrateCoMAndGetCoMVelocity(simDT, omega0.getDoubleValue(), desiredICPPosition, comPositionVector, comVelocityVector);

 
 desiredComPositionFramePoint.set(comPositionVector);
 desiredComVelocityFrameVector.set(comVelocityVector);

 desiredCOMvelAbsolute.set(Math.sqrt(Math.pow(desiredComVelocityFrameVector.getX(), 2) + Math.pow(desiredComVelocityFrameVector.getY(), 2)
         + Math.pow(desiredComVelocityFrameVector.getZ(), 2)));
 
   }

   public boolean isDone(double time)
   {
      computeTimeInStateAndEstimatedTimeRemaining(time);

      return (estimatedTimeRemainingForState.getDoubleValue() <= -1e-4);
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
      doubleSupportInitialTransferDuration.set(0.6);
      atAStop.set(true);

      footLocationList.clear();
      footLocationList.add(constantFootCenterCentersOfPressure.get(0).getFramePointCopy());
      footLocationList.add(constantFootCenterCentersOfPressure.get(1).getFramePointCopy());

      double doubleSupportInitialTransferDuration = 0.4;

      Point3d initialICPPosition = new Point3d();
      Vector3d initialICPVelocity = new Vector3d();
      Point3d InitialECMPPosition = new Point3d();

      computeICPPositionAndVelocity(initialICPPosition, initialICPVelocity, InitialECMPPosition, time);

      initializeDoubleSupportInitialTransfer(footLocationList, initialICPPosition, singleSupportDuration.getDoubleValue(),
              doubleSupportDuration.getDoubleValue(), doubleSupportInitialTransferDuration, omega0.getDoubleValue(), time);


   }


   public static ArrayList<Point3d> convertToListOfPoint3ds(ArrayList<YoFramePoint> yoFramePoints)
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
      return constantFootCenterCentersOfPressure;
   }

   public List<YoFramePoint> getFootCenterICPCornerPoints()
   {
      return footCenterICPCornerFramePoints;
   }

   public List<YoFramePoint> getToeICPCornerPoints()
   {
      return toeICPCornerFramePoints;
   }

   public Point3d getUpcomingCornerPoint()
   {
      return upcomingCornerPoint;
   }

}
