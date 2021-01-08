package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DMatrix1Row;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.matrixlib.MatrixTools;

public class LinearCoMTrajectoryHandler
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final DMatrixRMaj coefficientArray = new DMatrixRMaj(0, 3);

   private final FramePoint3D firstCoefficient = new FramePoint3D();
   private final FramePoint3D secondCoefficient = new FramePoint3D();
   private final FramePoint3D thirdCoefficient = new FramePoint3D();
   private final FramePoint3D fourthCoefficient = new FramePoint3D();
   private final FramePoint3D fifthCoefficient = new FramePoint3D();
   private final FramePoint3D sixthCoefficient = new FramePoint3D();

   private final CoMTrajectoryPlannerIndexHandler indexHandler;

   private boolean hasTrajectory = false;
   private int activeSegment = -1;

   public LinearCoMTrajectoryHandler(CoMTrajectoryPlannerIndexHandler indexHandler)
   {
      this.indexHandler = indexHandler;
   }

   public void clearTrajectory()
   {
      hasTrajectory = false;
      activeSegment = -1;
   }

   public boolean hasTrajectory()
   {
      return hasTrajectory;
   }

   public void setLinear(FramePoint3DReadOnly start, FramePoint3DReadOnly end, double duration)
   {
      coefficientArray.reshape(6, 3);
      coefficientArray.zero();

      int startIndex = indexHandler.getContactSequenceStartIndex(0);

      for (Axis3D axis : Axis3D.values)
      {
         int ordinal = axis.ordinal();
         double initial = start.getElement(ordinal);
         double rate = (end.getElement(ordinal) - initial) / duration;
         coefficientArray.set(startIndex + 4, ordinal, rate);
         coefficientArray.set(startIndex + 5, ordinal, initial);
      }

      activeSegment = 0;
      hasTrajectory = true;
   }

   public void setCoefficientsFromSolution(DMatrix1Row xSolution, DMatrix1Row ySolution, DMatrix1Row zSolution)
   {
      if (xSolution.getNumCols() != 1 || ySolution.getNumCols() != 1 || zSolution.getNumCols() != 1)
         throw new IllegalArgumentException("Solution vectors don't match in size");
      if (xSolution.getNumRows() != ySolution.getNumRows() || xSolution.getNumRows() != zSolution.getNumRows())
         throw new IllegalArgumentException("Solution vectors don't match in size");

      int numRows = xSolution.getNumRows();

      coefficientArray.reshape(numRows, 3);

      MatrixTools.setMatrixBlock(coefficientArray, 0, 0, xSolution, 0, 0, numRows, 1, 1.0);
      MatrixTools.setMatrixBlock(coefficientArray, 0, 1, ySolution, 0, 0, numRows, 1, 1.0);
      MatrixTools.setMatrixBlock(coefficientArray, 0, 2, zSolution, 0, 0, numRows, 1, 1.0);

      activeSegment = -1;
      hasTrajectory = true;
   }

   public void setActiveCoefficientsForSegment(int segmentId)
   {
      if (segmentId == activeSegment)
         return;

      int startIndex = indexHandler.getContactSequenceStartIndex(segmentId);
      int secondCoefficientIndex = startIndex + 1;
      int thirdCoefficientIndex = startIndex + 2;
      int fourthCoefficientIndex = startIndex + 3;
      int fifthCoefficientIndex = startIndex + 4;
      int sixthCoefficientIndex = startIndex + 5;

      for (Axis3D axis : Axis3D.values)
      {
         int ordinal = axis.ordinal();
         firstCoefficient.setElement(ordinal, coefficientArray.get(startIndex, ordinal));
         secondCoefficient.setElement(ordinal, coefficientArray.get(secondCoefficientIndex, ordinal));
         thirdCoefficient.setElement(ordinal, coefficientArray.get(thirdCoefficientIndex, ordinal));
         fourthCoefficient.setElement(ordinal, coefficientArray.get(fourthCoefficientIndex, ordinal));
         fifthCoefficient.setElement(ordinal, coefficientArray.get(fifthCoefficientIndex, ordinal));
         sixthCoefficient.setElement(ordinal, coefficientArray.get(sixthCoefficientIndex, ordinal));
      }

      activeSegment = segmentId;
      hasTrajectory = true;
   }

   public void computeCoMPosition(double omega, double timeInPhase, FixedFramePoint3DBasics comPositionToPack)
   {
      CoMTrajectoryPlannerTools.constructDesiredCoMPosition(comPositionToPack, firstCoefficient, secondCoefficient, thirdCoefficient, fourthCoefficient, fifthCoefficient,
                                                            sixthCoefficient, timeInPhase, omega);
   }

   public void computeCoMVelocity(double omega, double timeInPhase, FixedFrameVector3DBasics comVelocityToPack)
   {
      CoMTrajectoryPlannerTools.constructDesiredCoMVelocity(comVelocityToPack, firstCoefficient, secondCoefficient, thirdCoefficient, fourthCoefficient, fifthCoefficient,
                                                            sixthCoefficient, timeInPhase, omega);
   }

   public void computeCoMAcceleration(double omega, double timeInPhase, FixedFrameVector3DBasics comAccelerationToPack)
   {
      CoMTrajectoryPlannerTools.constructDesiredCoMAcceleration(comAccelerationToPack, firstCoefficient, secondCoefficient, thirdCoefficient, fourthCoefficient, fifthCoefficient,
                                                                sixthCoefficient, timeInPhase, omega);
   }

   public void computeVRPVelocity(double omega, double timeInPhase, FixedFrameVector3DBasics vrpVelocityToPack)
   {
      CoMTrajectoryPlannerTools.constructDesiredVRPVelocity(vrpVelocityToPack, firstCoefficient, secondCoefficient, thirdCoefficient, fourthCoefficient, fifthCoefficient,
                                                            sixthCoefficient, timeInPhase, omega);
   }

   public void compute(double omega, double timeInPhase, FixedFramePoint3DBasics comPositionToPack, FixedFrameVector3DBasics comVelocityToPack,
                       FixedFrameVector3DBasics comAccelerationToPack, FixedFramePoint3DBasics dcmPositionToPack, FixedFrameVector3DBasics dcmVelocityToPack,
                       FixedFramePoint3DBasics vrpPositionToPack, FixedFrameVector3DBasics vrpVelocityToPack)
   {

      computeCoMPosition(omega, timeInPhase, comPositionToPack);
      computeCoMVelocity(omega, timeInPhase, comVelocityToPack);
      computeCoMAcceleration(omega, timeInPhase, comAccelerationToPack);
      computeVRPVelocity(omega, timeInPhase, vrpVelocityToPack);

      CapturePointTools.computeCapturePointPosition(comPositionToPack, comVelocityToPack, omega, dcmPositionToPack);
      CapturePointTools.computeCapturePointVelocity(comVelocityToPack, comAccelerationToPack, omega, dcmVelocityToPack);
      CapturePointTools.computeCentroidalMomentumPivot(dcmPositionToPack, dcmVelocityToPack, omega, vrpPositionToPack);
   }
}
