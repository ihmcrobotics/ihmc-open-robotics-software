package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commons.lists.RecyclingArrayList;

/**
 * <p>
 * This command is designed to allow minimizing the tracking error along the trajectory segment indicated by {@link #segmentNumber}.
 * </p>
 *
 * <p>
 * Specifically, the command is designed to set up
 * </p>
 * <p> 0 = A &Theta; + B c + C </p>
 * <p>
 * where
 *    <ul>
 *    <li> &Theta; is the variable for the orientation variables at the beginning of the segment</li>
 *    <li> c are the continuous coefficient variables</li>
 *    <li> A, B, and C are the matrices at the i<sup>th</sup>  index of by {@link #AMatricesInSegment}, {@link #BMatricesInSegment},
 *    and {@link #CMatricesInSegment}, respectively. </li>
 *    </ul>
 * </p>
 */
public class OrientationTrajectoryCommand implements MPCCommand<OrientationTrajectoryCommand>
{
   private int commandId = -1;

   private int segmentNumber = -1;

   private final RecyclingArrayList<DMatrixRMaj> AMatricesInSegment = new RecyclingArrayList<>(() -> new DMatrixRMaj(6, 6));
   private final RecyclingArrayList<DMatrixRMaj> BMatricesInSegment = new RecyclingArrayList<>(() -> new DMatrixRMaj(6, 0));
   private final RecyclingArrayList<DMatrixRMaj> CMatricesInSegment = new RecyclingArrayList<>(() -> new DMatrixRMaj(6, 1));

   private double angleErrorMinimizationWeight;
   private double velocityErrorMinimizationWeight;

   public MPCCommandType getCommandType()
   {
      return MPCCommandType.ORIENTATION_TRAJECTORY;
   }

   @Override
   public void setCommandId(int id)
   {
      this.commandId = id;
   }

   @Override
   public int getCommandId()
   {
      return commandId;
   }

   public void reset()
   {
      angleErrorMinimizationWeight = 0.0;
      velocityErrorMinimizationWeight = 0.0;
      segmentNumber = -1;
      AMatricesInSegment.clear();
      BMatricesInSegment.clear();
      CMatricesInSegment.clear();
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

   public void setAngleErrorMinimizationWeight(double angleErrorMinimizationWeight)
   {
      this.angleErrorMinimizationWeight = angleErrorMinimizationWeight;
   }

   public void setVelocityErrorMinimizationWeight(double velocityErrorMinimizationWeight)
   {
      this.velocityErrorMinimizationWeight = velocityErrorMinimizationWeight;
   }

   public double getAngleErrorMinimizationWeight()
   {
      return angleErrorMinimizationWeight;
   }

   public double getVelocityErrorMinimizationWeight()
   {
      return velocityErrorMinimizationWeight;
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

   @Override
   public void set(OrientationTrajectoryCommand other)
   {
      reset();
      setCommandId(other.getCommandId());
      setSegmentNumber(other.getSegmentNumber());
      setAngleErrorMinimizationWeight(other.getAngleErrorMinimizationWeight());
      setVelocityErrorMinimizationWeight(other.getVelocityErrorMinimizationWeight());
      for (int tick = 0; tick < other.getNumberOfTicksInSegment(); tick++)
      {
         addAMatrix().set(other.getAMatrix(tick));
         addBMatrix().set(other.getBMatrix(tick));
         addCMatrix().set(other.getCMatrix(tick));
      }
   }
}
