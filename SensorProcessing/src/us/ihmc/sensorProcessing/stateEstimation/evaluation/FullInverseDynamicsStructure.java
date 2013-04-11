package us.ihmc.sensorProcessing.stateEstimation.evaluation;

import us.ihmc.sensorProcessing.stateEstimation.OrientationEstimator;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

public class FullInverseDynamicsStructure
{
   private final TwistCalculator twistCalculator;
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;
   private final RigidBody estimationLink;
   private final ReferenceFrame estimationFrame;
   private final RigidBody elevator;
   private final SixDoFJoint rootInverseDynamicsJoint;

   // TODO: What's a good name for this?
   public FullInverseDynamicsStructure(RigidBody elevator, RigidBody estimationLink, SixDoFJoint rootInverseDynamicsJoint)
   {
      this.elevator = elevator;
      this.rootInverseDynamicsJoint = rootInverseDynamicsJoint;

      twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), elevator);
      spatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, twistCalculator, 0.0, false);

      this.estimationLink = estimationLink;
      estimationFrame = estimationLink.getParentJoint().getFrameAfterJoint();
   }

   public SixDoFJoint getRootInverseDynamicsJoint()
   {
      return rootInverseDynamicsJoint;
   }

   public TwistCalculator getTwistCalculator()
   {
      return twistCalculator;
   }

   public SpatialAccelerationCalculator getSpatialAccelerationCalculator()
   {
      return spatialAccelerationCalculator;
   }

   public RigidBody getEstimationLink()
   {
      return estimationLink;
   }

   public ReferenceFrame getEstimationFrame()
   {
      return estimationFrame;
   }

   public RigidBody getElevator()
   {
      return elevator;
   }

   public void updateInternalState()
   {
      twistCalculator.compute();
      spatialAccelerationCalculator.compute();
   }

   public void updateRootJointBasedOnEstimator(OrientationEstimator estimator)
   {
      FrameOrientation estimatedOrientation = estimator.getEstimatedOrientation();
      FrameVector estimatedAngularVelocity = estimator.getEstimatedAngularVelocity();

      updateRootJointBasedOnEstimator(estimatedOrientation, estimatedAngularVelocity);
   }

   public void updateRootJointBasedOnEstimator(FrameOrientation estimatedOrientation, FrameVector estimatedAngularVelocity)
   {
      rootInverseDynamicsJoint.setRotation(estimatedOrientation.getQuaternion());

      elevator.updateFramesRecursively();

      ReferenceFrame elevatorFrame = rootInverseDynamicsJoint.getFrameBeforeJoint();
      ReferenceFrame bodyFrame = rootInverseDynamicsJoint.getFrameAfterJoint();

      estimatedAngularVelocity.changeFrame(bodyFrame);

      Twist bodyTwist = new Twist(bodyFrame, elevatorFrame, bodyFrame);
      bodyTwist.setAngularPart(estimatedAngularVelocity.getVector());
      rootInverseDynamicsJoint.setJointTwist(bodyTwist);
   }


}
