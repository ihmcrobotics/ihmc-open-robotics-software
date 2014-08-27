package us.ihmc.commonWalkingControlModules.kinematics;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.yoUtilities.YoVariableRegistry;

import com.yobotics.simulationconstructionset.DoubleYoVariable;

public class SpatialAccelerationProjector
{
   private final DoubleYoVariable projectionFactor;

   public SpatialAccelerationProjector(String namePrefix, YoVariableRegistry registry)
   {
      projectionFactor = new DoubleYoVariable(namePrefix + "ProjectionFactor", registry);
   }

   public void projectAcceleration(SpatialAccelerationVector accelerationToProject, FrameLineSegment2d closestEdge)
   {
      if (closestEdge == null)
         throw new RuntimeException("closestEdge == null");

      FrameVector2d axisOfRotation2d = closestEdge.getVectorCopy();
      FrameVector axisOfRotation = new FrameVector(axisOfRotation2d.getReferenceFrame(), axisOfRotation2d.getX(), axisOfRotation2d.getY(), 0.0);
      FramePoint offset = closestEdge.getFirstEndPointCopy().toFramePoint();

      ReferenceFrame baseFrame = accelerationToProject.getBaseFrame();
      ReferenceFrame bodyFrame = accelerationToProject.getBodyFrame();
      ReferenceFrame expressedInFrame = accelerationToProject.getExpressedInFrame();

      axisOfRotation.changeFrame(bodyFrame);
      offset.changeFrame(bodyFrame);

      Twist motionSubspaceTwist = new Twist(bodyFrame, baseFrame, bodyFrame, 1.0, 0.0, axisOfRotation.getVector(), offset.getVectorCopy());
      motionSubspaceTwist.changeFrame(expressedInFrame);

      DenseMatrix64F motionSubspaceMatrix = motionSubspaceTwist.toMatrix();
      DenseMatrix64F footAccelerationMatrix = accelerationToProject.toMatrix();
      DenseMatrix64F projectionFactors = new DenseMatrix64F(motionSubspaceMatrix.getNumCols(), footAccelerationMatrix.getNumCols());

      CommonOps.multTransA(motionSubspaceMatrix, footAccelerationMatrix, projectionFactors);
      CommonOps.mult(motionSubspaceMatrix, projectionFactors, footAccelerationMatrix);
      accelerationToProject.set(bodyFrame, baseFrame, expressedInFrame, footAccelerationMatrix, 0);
      projectionFactor.set(projectionFactors.get(0));
   }
}
