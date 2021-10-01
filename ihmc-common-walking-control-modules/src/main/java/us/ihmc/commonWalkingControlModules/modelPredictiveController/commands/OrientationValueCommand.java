package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;

/**
 * <p>
 * This command is designed to allow setting a transformation of the orientation value
 * at the beginning of the segment indicated by {@link #segmentNumber}.
 * </p>
 *
 * <p>
 * Specifically, the command is designed to set up
 * </p>
 * <p> value = A &Theta; + B c + C </p>
 * <p>
 *    where
 *    <ul>
 *    <li> value is specified by {@link #objectiveValue}</li>
 *    <li> &Theta; is the variable for the orientation variables at the beginning of the segment</li>
 *    <li> c are the continuous coefficient variables</li>
 *    <li> A, B, and C are the matrices set by {@link #AMatrix}, {@link #BMatrix}, and {@link #CMatrix}, respectively. </li>
 *    </ul>
 * </p>
 */
public class OrientationValueCommand implements MPCCommand<OrientationValueCommand>
{
   private int commandId = -1;
   private int segmentNumber = -1;

   private ConstraintType constraintType = ConstraintType.OBJECTIVE;

   private final DMatrixRMaj AMatrix = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj BMatrix = new DMatrixRMaj(6, 0);
   private final DMatrixRMaj CMatrix = new DMatrixRMaj(6, 1);

   private final DMatrixRMaj objectiveValue = new DMatrixRMaj(6, 1);

   private boolean useWeightScalar;
   private double objectiveWeight;
   private final DMatrixRMaj weightMatrix = new DMatrixRMaj(6, 6);

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
      return MPCCommandType.ORIENTATION_VALUE;
   }

   public void reset()
   {
      setSegmentNumber(-1);
      getAMatrix().zero();
      getBMatrix().zero();
      getCMatrix().zero();
      getObjectiveValue().zero();
      useWeightScalar = true;
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

   public DMatrixRMaj getObjectiveValue()
   {
      return objectiveValue;
   }

   public void setObjectiveValue(DMatrixRMaj objectiveValue)
   {
      this.objectiveValue.set(objectiveValue);
   }

   public void setObjectiveWeight(double objectiveWeight)
   {
      this.objectiveWeight = objectiveWeight;
   }

   public double getObjectiveWeight()
   {
      return objectiveWeight;
   }

   public void setUseWeightScalar(boolean useWeightScalar)
   {
      this.useWeightScalar = useWeightScalar;
   }

   public boolean useWeightScalar()
   {
      return useWeightScalar;
   }

   public void setWeightMatrix(DMatrixRMaj weightMatrix)
   {
      this.weightMatrix.set(weightMatrix);
   }

   public DMatrixRMaj getWeightMatrix()
   {
      return weightMatrix;
   }

   public void set(OrientationValueCommand other)
   {
      setCommandId(other.getCommandId());
      setSegmentNumber(other.getSegmentNumber());
      setAMatrix(other.getAMatrix());
      setBMatrix(other.getBMatrix());
      setCMatrix(other.getCMatrix());
      setObjectiveValue(other.getObjectiveValue());
      setUseWeightScalar(other.useWeightScalar());
      setObjectiveWeight(other.getObjectiveWeight());
      setWeightMatrix(other.getWeightMatrix());
   }
}
