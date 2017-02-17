package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import static us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels.HARD_CONSTRAINT;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;
import us.ihmc.robotics.screwTheory.Twist;

public class SpatialAccelerationCommand implements InverseDynamicsCommand<SpatialAccelerationCommand>
{
   private boolean hasWeight;
   private final SpatialAccelerationVector spatialAcceleration = new SpatialAccelerationVector();
   private final DenseMatrix64F weightVector = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);
   private final DenseMatrix64F selectionMatrix = CommonOps.identity(SpatialAccelerationVector.SIZE);

   private RigidBody base;
   private RigidBody endEffector;
   private RigidBody optionalPrimaryBase;

   private String baseName;
   private String endEffectorName;
   private String optionalPrimaryBaseName;

   /**
    * It refers to how important this task is:
    * <li> &alpha;=0 => the task will be ignored,
    * <li> &alpha;=1 => (default usage) the solver will try its best to achieve the command. </li>
    * It is useful for doing task priority without changing the actual weight in the solver.
    */
   private double alphaTaskPriority = 1.0;

   public SpatialAccelerationCommand()
   {
      removeWeight();
      resetAlphaTaskPriority();
   }

   public SpatialAccelerationCommand(long jacobianId)
   {
      this();
   }

   public void set(RigidBody base, RigidBody endEffector)
   {
      setBase(base);
      setEndEffector(endEffector);
   }

   public void setBase(RigidBody base)
   {
      this.base = base;
      baseName = base.getName();
   }

   public void setEndEffector(RigidBody endEffector)
   {
      this.endEffector = endEffector;
      endEffectorName = endEffector.getName();
   }

   public void setPrimaryBase(RigidBody primaryBase)
   {
      optionalPrimaryBase = primaryBase;
      optionalPrimaryBaseName = primaryBase.getName();
   }

   public void setWeight(double weight)
   {
      for(int i = 0; i < SpatialAccelerationVector.SIZE; i++)
         weightVector.set(i, 0, weight);
      hasWeight = weight != HARD_CONSTRAINT;
   }

   public void setWeight(double angular, double linear)
   {
      for(int i = 0; i < 3; i++)
         weightVector.set(i, 0, angular);
      for(int i = 3; i < SpatialAccelerationVector.SIZE; i++)
         weightVector.set(i, 0, linear);
      hasWeight = angular != HARD_CONSTRAINT && linear != HARD_CONSTRAINT;
   }

   public void setWeights(DenseMatrix64F weight)
   {
      hasWeight = true;

      for(int i = 0; i < SpatialAccelerationVector.SIZE; i++)
      {
         weightVector.set(i, 0, weight.get(i, 0));
         if (weight.get(i, 0) == HARD_CONSTRAINT)
            hasWeight = false;
      }
   }

   public void setAngularWeights(Vector3D angular)
   {
      weightVector.set(0, 0, angular.getX());
      weightVector.set(1, 0, angular.getY());
      weightVector.set(2, 0, angular.getZ());

      hasWeight = angular.getX() != HARD_CONSTRAINT && angular.getY() != HARD_CONSTRAINT && angular.getZ() != HARD_CONSTRAINT;
   }

   public void setWeights(Vector3D angular, Vector3D linear)
   {
      weightVector.set(0, 0, angular.getX());
      weightVector.set(1, 0, angular.getY());
      weightVector.set(2, 0, angular.getZ());
      weightVector.set(3, 0, linear.getX());
      weightVector.set(4, 0, linear.getY());
      weightVector.set(5, 0, linear.getZ());

      hasWeight = angular.getX() != HARD_CONSTRAINT && angular.getY() != HARD_CONSTRAINT && angular.getZ() != HARD_CONSTRAINT;
      hasWeight = linear.getX() != HARD_CONSTRAINT && linear.getY() != HARD_CONSTRAINT && linear.getZ() != HARD_CONSTRAINT && hasWeight;
   }

   public void setLinearWeightsToZero()
   {
      for (int i = 3; i < SpatialAccelerationVector.SIZE; i++)
         weightVector.set(i, 0, 0.0);
   }

   public void setAlphaTaskPriority(double alpha)
   {
      alphaTaskPriority = MathTools.clipToMinMax(alpha, 0.0, 1.0);
   }

   public void setSpatialAcceleration(SpatialAccelerationVector spatialAcceleration)
   {
      this.spatialAcceleration.set(spatialAcceleration);
   }

   public void setAngularAcceleration(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, FrameVector desiredAngularAcceleration)
   {
      spatialAcceleration.setToZero(bodyFrame, baseFrame, desiredAngularAcceleration.getReferenceFrame());
      spatialAcceleration.setAngularPart(desiredAngularAcceleration.getVector());
   }

   public void setLinearAcceleration(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, FrameVector desiredLinearAcceleration)
   {
      spatialAcceleration.setToZero(bodyFrame, baseFrame, desiredLinearAcceleration.getReferenceFrame());
      spatialAcceleration.setLinearPart(desiredLinearAcceleration.getVector());
      spatialAcceleration.changeFrameNoRelativeMotion(bodyFrame);
   }

   @Override
   public void set(SpatialAccelerationCommand other)
   {
      hasWeight = other.hasWeight;
      setWeights(other.getWeightVector());
      alphaTaskPriority = other.alphaTaskPriority;

      spatialAcceleration.set(other.getSpatialAcceleration());
      selectionMatrix.set(other.getSelectionMatrix());
      base = other.getBase();
      endEffector = other.getEndEffector();
      baseName = other.baseName;
      endEffectorName = other.endEffectorName;

      optionalPrimaryBase = other.optionalPrimaryBase;
      optionalPrimaryBaseName = other.optionalPrimaryBaseName;
   }

   public void setSelectionMatrixToIdentity()
   {
      selectionMatrix.reshape(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
      CommonOps.setIdentity(selectionMatrix);
   }

   public void setSelectionMatrixForLinearControl()
   {
      selectionMatrix.reshape(3, Twist.SIZE);
      selectionMatrix.zero();
      selectionMatrix.set(0, 3, 1.0);
      selectionMatrix.set(1, 4, 1.0);
      selectionMatrix.set(2, 5, 1.0);
   }

   public void setSelectionMatrixForAngularControl()
   {
      selectionMatrix.reshape(3, Twist.SIZE);
      selectionMatrix.zero();
      selectionMatrix.set(0, 0, 1.0);
      selectionMatrix.set(1, 1, 1.0);
      selectionMatrix.set(2, 2, 1.0);
   }

   public void setSelectionMatrix(DenseMatrix64F selectionMatrix)
   {
      if (selectionMatrix.getNumRows() > SpatialAccelerationVector.SIZE)
         throw new RuntimeException("Unexpected number of rows: " + selectionMatrix.getNumRows());
      if (selectionMatrix.getNumCols() != SpatialAccelerationVector.SIZE)
         throw new RuntimeException("Unexpected number of columns: " + selectionMatrix.getNumCols());

      this.selectionMatrix.set(selectionMatrix);
   }

   public boolean getHasWeight()
   {
      return hasWeight;
   }

   public void getWeightMatrix(DenseMatrix64F weightMatrixToPack)
   {
      weightMatrixToPack.reshape(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
      CommonOps.setIdentity(weightMatrixToPack);
      for (int i = 0; i < SpatialAccelerationVector.SIZE; i++)
         weightMatrixToPack.set(i, i, weightVector.get(i, 0));
   }

   public DenseMatrix64F getWeightVector()
   {
      return weightVector;
   }

   public SpatialAccelerationVector getSpatialAcceleration()
   {
      return spatialAcceleration;
   }

   public DenseMatrix64F getSelectionMatrix()
   {
      return selectionMatrix;
   }

   public RigidBody getBase()
   {
      return base;
   }

   public String getBaseName()
   {
      return baseName;
   }

   public RigidBody getEndEffector()
   {
      return endEffector;
   }

   public String getEndEffectorName()
   {
      return endEffectorName;
   }

   public RigidBody getPrimaryBase()
   {
      return optionalPrimaryBase;
   }

   public String getPrimaryBaseName()
   {
      return optionalPrimaryBaseName;
   }

   public double getAlphaTaskPriority()
   {
      return alphaTaskPriority;
   }

   public void removeWeight()
   {
      setWeight(HARD_CONSTRAINT);
   }

   public void resetAlphaTaskPriority()
   {
      alphaTaskPriority = 1.0;
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.TASKSPACE;
   }

   @Override
   public String toString()
   {
      String ret = getClass().getSimpleName() + ": base = " + base.getName() + "endEffector = " + endEffector.getName() + ", spatialAcceleration = " + spatialAcceleration;
      return ret;
   }
}
