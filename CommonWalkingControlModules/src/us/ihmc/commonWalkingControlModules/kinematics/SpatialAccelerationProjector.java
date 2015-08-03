package us.ihmc.commonWalkingControlModules.kinematics;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;


public class SpatialAccelerationProjector
{
   private final DoubleYoVariable projectionFactor;

   public SpatialAccelerationProjector(String namePrefix, YoVariableRegistry registry)
   {
      projectionFactor = new DoubleYoVariable(namePrefix + "ProjectionFactor", registry);
   }

   private final FrameVector axisOfRotation = new FrameVector();
   private final FrameVector2d axisOfRotation2d = new FrameVector2d();

   public void projectAcceleration(SpatialAccelerationVector accelerationToProject, FrameLineSegment2d closestEdge)
   {
      if (closestEdge == null)
         throw new RuntimeException("closestEdge == null");

      closestEdge.getFrameVector(axisOfRotation2d);
      axisOfRotation.setXYIncludingFrame(axisOfRotation2d);
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
