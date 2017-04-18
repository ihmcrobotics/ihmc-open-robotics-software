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

   public final DenseMatrix64F positionOffset;

   public final DenseMatrix64F Aineq;
   public final DenseMatrix64F bineq;

   private int indexOfVariableToConstrain;

   private double deltaInside = 0.0;

   private final ConvexPolygon2d convexPolygon = new ConvexPolygon2d();

   public ConstraintToConvexRegion(int maximumNumberOfVertices)
   {
      CommonOps.scale(-1.0, indexSelectionMatrix);

      positionOffset = new DenseMatrix64F(2, 1);

      Aineq = new DenseMatrix64F(maximumNumberOfVertices, maximumNumberOfVertices);
      bineq = new DenseMatrix64F(maximumNumberOfVertices, 1);
   }

   public void reset()
   {
      positionOffset.zero();
      convexPolygon.clear();

      Aineq.zero();
      bineq.zero();
   }

   public void addVertex(FramePoint2d vertex)
   {
      vertex.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      convexPolygon.addVertex(vertex.getPoint());
   }

   public void addVertex(FramePoint vertex)
   {
      vertex.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      convexPolygon.addVertex(vertex.getX(), vertex.getY());
   }

   public void setPolygon()
   {
      convexPolygon.update();
   }

   public void setIndexOfVariableToConstrain(int index)
   {
      indexOfVariableToConstrain = index;
   }


   public void setPositionOffset(DenseMatrix64F offset)
   {
      MatrixTools.setMatrixBlock(positionOffset, 0, 0, offset, 0, 0, 2, 1, 1.0);
   }

   public void setDeltaInside(double deltaInside)
   {
      this.deltaInside = deltaInside;
   }

   private void reshape()
   {
      int numberOfVertices = convexPolygon.getNumberOfVertices();
      Aineq.reshape(numberOfVertices, numberOfVertices);
      bineq.reshape(numberOfVertices, 1);
   }

   public void formulateConstraint()
   {
      reshape();

      PolygonWiggler.convertToInequalityConstraints(convexPolygon, Aineq, bineq, deltaInside);
      CommonOps.multAdd(-1.0, Aineq, positionOffset, bineq);
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
