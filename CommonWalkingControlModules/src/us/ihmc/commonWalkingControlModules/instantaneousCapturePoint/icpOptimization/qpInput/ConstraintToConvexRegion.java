package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.qpInput;

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
   public final DenseMatrix64F indexSelectionMatrix = CommonOps.identity(2, 2);

   public final DenseMatrix64F dynamics_beq;

   public final DenseMatrix64F Aineq;
   public final DenseMatrix64F bineq;

   private int indexOfVariableToConstrain;
   private int numberOfVertices = 0;

   private double vertexMinimum = 0.0;

   private final ConvexPolygon2d convexPolygon = new ConvexPolygon2d();

   public ConstraintToConvexRegion(int maximumNumberOfVertices)
   {
      CommonOps.scale(-1.0, indexSelectionMatrix);

      dynamics_beq = new DenseMatrix64F(2, 1);

      Aineq = new DenseMatrix64F(maximumNumberOfVertices, maximumNumberOfVertices);
      bineq = new DenseMatrix64F(maximumNumberOfVertices, 1);
   }

   public void reset()
   {
      dynamics_beq.zero();
      convexPolygon.clear();

      Aineq.zero();
      bineq.zero();

      numberOfVertices = 0;
   }

   private void reshape()
   {
      Aineq.reshape(numberOfVertices, numberOfVertices);
      bineq.reshape(numberOfVertices, 1);
   }

   public void setIndexOfVariableToConstrain(int index)
   {
      indexOfVariableToConstrain = index;
   }

   public void addVertex(FramePoint2d vertex)
   {
      vertex.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      convexPolygon.addVertex(vertex.getPoint());
      numberOfVertices++;
   }

   public void addVertex(FramePoint vertex)
   {
      vertex.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      convexPolygon.addVertex(vertex.getX(), vertex.getY());
      numberOfVertices++;
   }

   public void setPositionOffset(DenseMatrix64F offset)
   {
      MatrixTools.setMatrixBlock(dynamics_beq, 0, 0, offset, 0, 0, 2, 1, 1.0);
   }

   public void setVertexMinimum(double vertexMinimum)
   {
      this.vertexMinimum = vertexMinimum;
   }

   public void formulateConstraint()
   {
      reshape();

      PolygonWiggler.convertToInequalityConstraints(convexPolygon, Aineq, bineq, vertexMinimum);
      CommonOps.multAdd(-1.0, Aineq, dynamics_beq, bineq);
   }

   public void setPolygon()
   {
      convexPolygon.update();
   }

   public int getNumberOfVertices()
   {
      return convexPolygon.getNumberOfVertices();
   }

   public int getIndexOfVariableToConstrain()
   {
      return indexOfVariableToConstrain;
   }
}
