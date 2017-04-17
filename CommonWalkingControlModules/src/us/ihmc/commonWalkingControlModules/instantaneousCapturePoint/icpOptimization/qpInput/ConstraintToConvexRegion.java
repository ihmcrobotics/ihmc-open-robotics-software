package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.qpInput;

import java.util.ArrayList;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.footstepPlanning.polygonWiggling.PolygonWiggler;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class ConstraintToConvexRegion
{
   public final DenseMatrix64F quadraticTerm;

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
   private double vertexMinimum = 0.0;

   private final ConvexPolygon2d convexPolygon = new ConvexPolygon2d();
   private final boolean useEfficientFormulation;

   public ConstraintToConvexRegion(int maximumNumberOfVertices, boolean useEfficientFormulation)
   {
      this.maximumNumberOfVertices = maximumNumberOfVertices;
      this.useEfficientFormulation = useEfficientFormulation;

      CommonOps.scale(-1.0, indexSelectionMatrix);

      dynamics_beq = new DenseMatrix64F(2, 1);
      if (!useEfficientFormulation)
      {
         quadraticTerm = new DenseMatrix64F(maximumNumberOfVertices, maximumNumberOfVertices);

         dynamics_Aeq = new DenseMatrix64F(maximumNumberOfVertices, 2);

         sum_Aeq = new DenseMatrix64F(maximumNumberOfVertices, 1);
         sum_beq = new DenseMatrix64F(1, 1);

         for (int i = 0; i < maximumNumberOfVertices; i++)
            vertexLocations.add(new DenseMatrix64F(1, 2));
      }
      else
      {
         quadraticTerm = null;

         dynamics_Aeq = null;

         sum_Aeq = null;
         sum_beq = null;
      }

      Aineq = new DenseMatrix64F(maximumNumberOfVertices, maximumNumberOfVertices);
      bineq = new DenseMatrix64F(maximumNumberOfVertices, 1);
   }

   public void reset()
   {
      dynamics_beq.zero();
      if (!useEfficientFormulation)
      {
         quadraticTerm.zero();

         dynamics_Aeq.zero();

         sum_Aeq.zero();
         sum_beq.zero();

         for (int i = 0; i < maximumNumberOfVertices; i++)
            vertexLocations.get(i).zero();
      }
      else
      {
         convexPolygon.clear();
      }

      Aineq.zero();
      bineq.zero();

      numberOfVertices = 0;
   }

   private void reshape()
   {
      if (!useEfficientFormulation)
      {
         quadraticTerm.reshape(numberOfVertices, numberOfVertices);

         dynamics_Aeq.reshape(numberOfVertices, 2);
         sum_Aeq.reshape(numberOfVertices, 1);
      }

      Aineq.reshape(numberOfVertices, numberOfVertices);
      bineq.reshape(numberOfVertices, 1);
   }

   public void setIndexOfVariableToConstrain(int index)
   {
      indexOfVariableToConstrain = index;
   }

   public void addVertex(DenseMatrix64F vertex)
   {
      if (!useEfficientFormulation)
         vertexLocations.get(numberOfVertices).set(vertex);

      numberOfVertices++;
   }

   private final DenseMatrix64F tmpPoint = new DenseMatrix64F(1, 2);
   public void addVertex(FramePoint2d vertex)
   {
      vertex.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      if (!useEfficientFormulation)
      {
         tmpPoint.zero();
         tmpPoint.set(0, 0, vertex.getX());
         tmpPoint.set(0, 1, vertex.getY());

         addVertex(tmpPoint);
      }
      else
      {
         convexPolygon.addVertex(vertex.getPoint());
         numberOfVertices++;
      }
   }

   public void addVertex(FramePoint vertex)
   {
      vertex.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      if (!useEfficientFormulation)
      {
         tmpPoint.zero();
         tmpPoint.set(0, 0, vertex.getX());
         tmpPoint.set(0, 1, vertex.getY());

         addVertex(tmpPoint);
      }
      else
      {
         convexPolygon.addVertex(vertex.getX(), vertex.getY());
         numberOfVertices++;
      }
   }

   public void setPositionOffset(DenseMatrix64F offset)
   {
      MatrixTools.setMatrixBlock(dynamics_beq, 0, 0, offset, 0, 0, 2, 1, 1.0);
   }

   public void setSmoothingWeight(double smoothingWeight)
   {
      this.smoothingWeight = smoothingWeight;
   }

   public void setVertexMinimum(double vertexMinimum)
   {
      this.vertexMinimum = vertexMinimum;
   }

   public void formulateConstraint()
   {
      reshape();

      if (!useEfficientFormulation)
      {
         for (int i = 0; i < numberOfVertices; i++)
         {
            dynamics_Aeq.set(i, 0, vertexLocations.get(i).get(0, 0));
            dynamics_Aeq.set(i, 1, vertexLocations.get(i).get(0, 1));

            sum_Aeq.set(i, 0, 1.0);

            Aineq.set(i, i, -1.0);
            bineq.set(i, 0, -vertexMinimum);
         }
         sum_beq.set(0, 0, 1.0);

         setSmoothingCost();
      }
      else
      {
         PolygonWiggler.convertToInequalityConstraints(convexPolygon, Aineq, bineq, vertexMinimum);
         CommonOps.multAdd(-1.0, Aineq, dynamics_beq, bineq);
      }
   }

   private void setSmoothingCost()
   {
      CommonOps.setIdentity(quadraticTerm);
      CommonOps.scale(smoothingWeight, quadraticTerm);
   }

   public void setPolygon()
   {
      if (useEfficientFormulation)
         convexPolygon.update();
   }

   public int getNumberOfVertices()
   {
      if (useEfficientFormulation)
         return convexPolygon.getNumberOfVertices();
      else
         return numberOfVertices;
   }

   public int getIndexOfVariableToConstrain()
   {
      return indexOfVariableToConstrain;
   }
}
