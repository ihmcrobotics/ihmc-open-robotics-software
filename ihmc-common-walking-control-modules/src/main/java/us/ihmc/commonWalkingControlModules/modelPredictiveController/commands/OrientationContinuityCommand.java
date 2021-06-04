package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;

/**
 * <p>
 * This command is designed to allow setting the transformation of the orientation value
 * at the beginning of the segment indicated by {@link #segmentNumber} to be equal to the value at beginning
 * of the next segment
 * </p>
 *
 * <p>
 * Specifically, the command is designed to set up
 * </p>
 * <p> &Theta;<sub>next</sub> = A &Theta;<sub>this</sub> + B c + C </p>
 * <p>
 *    where
 *    <ul>
 *    <li> &Theta;<sub>this</sub> is the variable for the orientation variables at the beginning of the specified segment</li>
 *    <li> &Theta;<sub>next</sub> is the variable for the orientation variables at the beginning of the next segment</li>
 *    <li> c are the continuous coefficient variables</li>
 *    <li> A, B, and C are the matrices set by {@link #AMatrix}, {@link #BMatrix}, and {@link #CMatrix}, respectively. </li>
 *    </ul>
 * </p>
 */
public class OrientationContinuityCommand implements MPCCommand<OrientationContinuityCommand>
{
   private int commandId = -1;
   private int segmentNumber = -1;

   private ConstraintType constraintType = ConstraintType.EQUALITY;

   private final DMatrixRMaj AMatrix = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj BMatrix = new DMatrixRMaj(6, 0);
   private final DMatrixRMaj CMatrix = new DMatrixRMaj(6, 1);

   private double objectiveWeight;

   public void setCommandId(int commandId)
   {
      this.commandId = commandId;
   }
   public int getCommandId()
   {
      return commandId;
   }

   public MPCCommandType getCommandType()
   {
      return MPCCommandType.ORIENTATION_CONTINUITY;
   }

   public void reset()
   {
      setSegmentNumber(-1);
      getAMatrix().zero();
      getBMatrix().zero();
      getCMatrix().zero();
   }

   public int getSegmentNumber()
   {
      return segmentNumber;
   }

   public void setSegmentNumber(int segmentNumber)
   {
      this.segmentNumber = segmentNumber;
   }

   public ConstraintType getConstraintType()
   {
      return constraintType;
   }

   public void setConstraintType(ConstraintType constraintType)
   {
      this.constraintType = constraintType;
   }

   public DMatrixRMaj getAMatrix()
   {
      return AMatrix;
   }

   public void setAMatrix(DMatrixRMaj AMatrix)
   {
      this.AMatrix.set(AMatrix);
   }

   public DMatrixRMaj getBMatrix()
   {
      return BMatrix;
   }

   public void setBMatrix(DMatrixRMaj BMatrix)
   {
      this.BMatrix.set(BMatrix);
   }

   public DMatrixRMaj getCMatrix()
   {
      return CMatrix;
   }

   public void setCMatrix(DMatrixRMaj CMatrix)
   {
      this.CMatrix.set(CMatrix);
   }

   public void setObjectiveWeight(double objectiveWeight)
   {
      this.objectiveWeight = objectiveWeight;
   }

   public double getObjectiveWeight()
   {
      return objectiveWeight;
   }

   public void set(OrientationContinuityCommand other)
   {
      setCommandId(other.getCommandId());
      setSegmentNumber(other.getSegmentNumber());
      setAMatrix(other.getAMatrix());
      setBMatrix(other.getBMatrix());
      setCMatrix(other.getCMatrix());
   }
}
