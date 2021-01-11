package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DMatrix1Row;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.matrixlib.MatrixTools;

import java.util.List;

public class LinearCoMTrajectoryHandler
{

   private final DMatrixRMaj coefficientArray = new DMatrixRMaj(0, 3);

   private final RecyclingArrayList<CoMTrajectory> trajectories = new RecyclingArrayList<>(CoMTrajectory::new);

   private boolean hasTrajectory = false;

   public void clearTrajectory()
   {
      trajectories.clear();
      hasTrajectory = false;
   }

   public boolean hasTrajectory()
   {
      return hasTrajectory;
   }

   public void setLinear(FramePoint3DReadOnly start, FramePoint3DReadOnly end, double duration)
   {
      trajectories.clear();
      double rateX = (end.getX() - start.getX()) / duration;
      double rateY = (end.getY() - start.getY()) / duration;
      double rateZ = (end.getZ() - start.getZ()) / duration;
      CoMTrajectory trajectory = trajectories.add();
      trajectory.setFifthCoefficient(ReferenceFrame.getWorldFrame(), rateX, rateY, rateZ);
      trajectory.setSixthCoefficient(ReferenceFrame.getWorldFrame(), start.getX(), start.getY(), start.getZ());

      hasTrajectory = true;
   }

   public void setCoefficientsFromSolution(List<? extends ContactStateProvider> contacts, DMatrix1Row xSolution, DMatrix1Row ySolution, DMatrix1Row zSolution)
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

      trajectories.clear();
      int startRow = 0;
      for (int i = 0; i < contacts.size(); i++)
      {
         CoMTrajectory trajectory = trajectories.add();
         trajectory.setCoefficients(coefficientArray, startRow);
         trajectory.set(contacts.get(i).getTimeInterval());
         startRow += CoMTrajectoryPlannerIndexHandler.polynomialCoefficientsPerSegment;
      }

      hasTrajectory = true;
   }

   public void computeCoMPosition(int segment, double omega, double timeInSegment, FixedFramePoint3DBasics comPositionToPack)
   {
      trajectories.get(segment).setOmega(omega);
      trajectories.get(segment).computeCoMPosition(timeInSegment, comPositionToPack);
   }

   public void computeCoMVelocity(int segment, double omega, double timeInSegment, FixedFrameVector3DBasics comVelocityToPack)
   {
      trajectories.get(segment).setOmega(omega);
      trajectories.get(segment).computeCoMVelocity(timeInSegment, comVelocityToPack);
   }

   public void computeCoMAcceleration(int segment, double omega, double timeInSegment, FixedFrameVector3DBasics comAccelerationToPack)
   {
      trajectories.get(segment).setOmega(omega);
      trajectories.get(segment).computeCoMAcceleration(timeInSegment, comAccelerationToPack);
   }

   public void computeVRPVelocity(int segment, double omega, double timeInSegment, FixedFrameVector3DBasics vrpVelocityToPack)
   {
      trajectories.get(segment).setOmega(omega);
      trajectories.get(segment).computeVRPVelocity(timeInSegment, vrpVelocityToPack);
   }

   public void compute(int segment,
                       double omega,
                       double timeInSegment,
                       FixedFramePoint3DBasics comPositionToPack,
                       FixedFrameVector3DBasics comVelocityToPack,
                       FixedFrameVector3DBasics comAccelerationToPack,
                       FixedFramePoint3DBasics dcmPositionToPack,
                       FixedFrameVector3DBasics dcmVelocityToPack,
                       FixedFramePoint3DBasics vrpPositionToPack,
                       FixedFrameVector3DBasics vrpVelocityToPack)
   {
      trajectories.get(segment).setOmega(omega);
      trajectories.get(segment).compute(timeInSegment, comPositionToPack, comVelocityToPack, comAccelerationToPack, dcmPositionToPack, dcmVelocityToPack, vrpPositionToPack, vrpVelocityToPack);
   }
}
