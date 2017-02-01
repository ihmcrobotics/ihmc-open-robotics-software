package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.qpInput;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import java.util.ArrayList;

public class ConstraintToConvexRegion
{
   public final DenseMatrix64F smoothingCost;
   public final DenseMatrix64F indexSelectionMatrix = CommonOps.identity(2, 2);

   public final DenseMatrix64F dynamics_Aeq;
   public final DenseMatrix64F dynamics_beq;

   public final DenseMatrix64F sum_Aeq;
   public final DenseMatrix64F sum_beq;

   public final DenseMatrix64F Aineq;
   public final DenseMatrix64F bineq;

   private final ArrayList<DenseMatrix64F> vertexLocations = new ArrayList<>();

   private final int maximumNumberOfVertices;
   private int indexOfVariableToConstrain;
   private int numberOfVertices = 0;

   private double smoothingWeight = 0.001;

   public ConstraintToConvexRegion(int maximumNumberOfVertices)
   {
      this.maximumNumberOfVertices = maximumNumberOfVertices;

      smoothingCost = new DenseMatrix64F(maximumNumberOfVertices, maximumNumberOfVertices);
      CommonOps.scale(-1.0, indexSelectionMatrix);

      dynamics_Aeq = new DenseMatrix64F(maximumNumberOfVertices, 2);
      dynamics_beq = new DenseMatrix64F(2, 1);

      sum_Aeq = new DenseMatrix64F(maximumNumberOfVertices, 1);
      sum_beq = new DenseMatrix64F(1, 1);

      Aineq = new DenseMatrix64F(maximumNumberOfVertices, maximumNumberOfVertices);
      bineq = new DenseMatrix64F(maximumNumberOfVertices, 1);

      for (int i = 0; i < maximumNumberOfVertices; i++)
         vertexLocations.add(new DenseMatrix64F(1, 2));
   }

   public void reset()
   {
      smoothingCost.zero();

      dynamics_Aeq.zero();
      dynamics_beq.zero();

      sum_Aeq.zero();
      sum_beq.zero();

      Aineq.zero();
      bineq.zero();

      numberOfVertices = 0;
      for (int i = 0; i < maximumNumberOfVertices; i++)
         vertexLocations.get(i).zero();
   }

   private void reshape()
   {
      smoothingCost.reshape(numberOfVertices, numberOfVertices);

      dynamics_Aeq.reshape(numberOfVertices, 2);
      sum_Aeq.reshape(numberOfVertices, 1);

      Aineq.reshape(numberOfVertices, numberOfVertices);
      bineq.reshape(numberOfVertices, 1);
   }

   public void setIndexOfVariableToConstrain(int index)
   {
      indexOfVariableToConstrain = index;
   }

   public void addVertex(DenseMatrix64F vertex)
   {
      vertexLocations.get(numberOfVertices).set(vertex);
      numberOfVertices++;
   }

   private final DenseMatrix64F tmpPoint = new DenseMatrix64F(1, 2);
   public void addVertex(FramePoint2d vertex)
   {
      vertex.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      tmpPoint.zero();
      tmpPoint.set(0, 0, vertex.getX());
      tmpPoint.set(0, 1, vertex.getY());

      addVertex(tmpPoint);
   }

   public void addVertex(FramePoint vertex)
   {
      vertex.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      tmpPoint.zero();
      tmpPoint.set(0, 0, vertex.getX());
      tmpPoint.set(0, 1, vertex.getY());

      addVertex(tmpPoint);
   }

   public void setPositionOffset(DenseMatrix64F offset)
   {
      MatrixTools.setMatrixBlock(dynamics_beq, 0, 0, offset, 0, 0, 2, 1, 1.0);
   }

   public void setSmoothingWeight(double smoothingWeight)
   {
      this.smoothingWeight = smoothingWeight;
   }

   public void formulateConstraint()
   {
      reshape();

      for (int i = 0; i < numberOfVertices; i++)
      {
         dynamics_Aeq.set(i, 0, vertexLocations.get(i).get(0, 0));
         dynamics_Aeq.set(i, 1, vertexLocations.get(i).get(0, 1));

         sum_Aeq.set(i, 0, 1.0);
         Aineq.set(i, i, -1.0);
      }
      sum_beq.set(0, 0, 1.0);

      CommonOps.setIdentity(smoothingCost);
      CommonOps.scale(smoothingWeight, smoothingCost);
   }

   public int getNumberOfVertices()
   {
      return numberOfVertices;
   }

   public int getIndexOfVariableToConstrain()
   {
      return indexOfVariableToConstrain;
   }
}
