package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

import java.util.Arrays;

public class OrientationTrajectoryCommand
{
   private int segmentNumber = -1;

   private final RecyclingArrayList<DMatrixRMaj> AMatricesInSegment = new RecyclingArrayList<>(() -> new DMatrixRMaj(6, 6));
   private final RecyclingArrayList<DMatrixRMaj> BMatricesInSegment = new RecyclingArrayList<>(() -> new DMatrixRMaj(6, 0));
   private final RecyclingArrayList<DMatrixRMaj> CMatricesInSegment = new RecyclingArrayList<>(() -> new DMatrixRMaj(6, 1));

   private final DMatrixRMaj initialError = new DMatrixRMaj(6, 1);

   public void reset()
   {
      segmentNumber = -1;
      AMatricesInSegment.clear();
      BMatricesInSegment.clear();
      CMatricesInSegment.clear();
      Arrays.fill(initialError.data, 0, 6, Double.NaN);
   }

   public DMatrixRMaj getInitialError()
   {
      return initialError;
   }

   public int getSegmentNumber()
   {
      return segmentNumber;
   }

   public void setSegmentNumber(int segmentNumber)
   {
      this.segmentNumber = segmentNumber;
   }

   public int getNumberOfTicksInSegment()
   {
      return AMatricesInSegment.size();
   }

   public void setInitialOrientationError(DMatrixRMaj initialError)
   {
      this.initialError.set(initialError);
   }

   public void setInitialOrientationError(FrameVector3DReadOnly initialOrientationError)
   {
      initialOrientationError.get(initialError);
   }

   public void setInitialOrientationVelocityErrorInBodyFrame(FrameVector3DReadOnly initialOrientationVelocityErrorInBodyFrame)
   {
      initialOrientationVelocityErrorInBodyFrame.get(3, initialError);
   }

   public DMatrixRMaj addAMatrix()
   {
      return AMatricesInSegment.add();
   }

   public DMatrixRMaj addBMatrix()
   {
      return BMatricesInSegment.add();
   }

   public DMatrixRMaj addCMatrix()
   {
      return CMatricesInSegment.add();
   }

   public DMatrixRMaj getAMatrix(int tick)
   {
      return AMatricesInSegment.get(tick);
   }

   public DMatrixRMaj getBMatrix(int tick)
   {
      return BMatricesInSegment.get(tick);
   }

   public DMatrixRMaj getCMatrix(int tick)
   {
      return CMatricesInSegment.get(tick);
   }

   public DMatrixRMaj getLastAMatrix()
   {
      return AMatricesInSegment.getLast();
   }

   public DMatrixRMaj getLastBMatrix()
   {
      return BMatricesInSegment.getLast();
   }

   public DMatrixRMaj getLastCMatrix()
   {
      return CMatricesInSegment.getLast();
   }
}
