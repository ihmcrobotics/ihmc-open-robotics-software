package us.ihmc.robotics.physics;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyAccelerationProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.MultiBodySystemStateIntegrator;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

public class FrameShapePosePredictor
{
   private final ForwardDynamicsCalculator forwardDynamicsCalculator;
   private final MultiBodySystemStateIntegrator integrator = new MultiBodySystemStateIntegrator();

   private final Pose3D pose = new Pose3D();
   private final Twist bodyTwist = new Twist();
   private final SpatialAcceleration bodyAcceleration = new SpatialAcceleration();

   public FrameShapePosePredictor(ForwardDynamicsCalculator forwardDynamicsCalculator)
   {
      this.forwardDynamicsCalculator = forwardDynamicsCalculator;
   }

   public FrameShape3DReadOnly predictShape(FrameShape3DReadOnly shape, RigidBodyReadOnly rigidBody, double dt)
   {
      if (rigidBody == null)
         return shape;

      ReferenceFrame shapeFrame = shape.getReferenceFrame();
      RigidBodyAccelerationProvider accelerationProvider = forwardDynamicsCalculator.getAccelerationProvider();
      integrator.setIntegrationDT(dt);
      bodyTwist.setIncludingFrame(rigidBody.getBodyFixedFrame().getTwistOfFrame());
      bodyTwist.changeFrame(shapeFrame);
      bodyTwist.setBodyFrame(shapeFrame);
      bodyAcceleration.setIncludingFrame(accelerationProvider.getAccelerationOfBody(rigidBody));
      bodyAcceleration.changeFrame(shapeFrame);
      bodyAcceleration.setBodyFrame(shapeFrame);

      pose.set(shapeFrame.getTransformToRoot());
      integrator.doubleIntegrate(bodyAcceleration, bodyTwist, pose);

      FrameShape3DBasics predictedShape = (FrameShape3DBasics) shape.copy();
      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame(shapeFrame.getName() + "Predicted", shapeFrame.getRootFrame());
      poseReferenceFrame.setPoseAndUpdate(pose);
      predictedShape.setReferenceFrame(poseReferenceFrame);
      return predictedShape;
   }
}
