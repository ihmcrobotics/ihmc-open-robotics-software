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

/**
 * This class predicts the pose of a frame shape by integrating the velocity and acceleration of the
 * rigid-body it is attached to.
 * 
 * @author Sylvain Bertrand
 */
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

   /**
    * Computes and returns the shape as predicted it will be {@code dt} seconds in the future.
    * <p>
    * The shape is assumed to be rigidly fixed to the rigid-body.
    * </p>
    * 
    * @param shape     the shape to predict the pose of. Not modified.
    * @param rigidBody the rigid-body the shape is rigidly attached to. Not modified.
    * @param dt        the integration time.
    * @return the shape expressed in the reference frame incorporating the time integration.
    */
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
