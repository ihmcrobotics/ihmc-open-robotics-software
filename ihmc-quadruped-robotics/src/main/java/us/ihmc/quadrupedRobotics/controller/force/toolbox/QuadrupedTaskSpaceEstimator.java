package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedTaskSpaceEstimator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final QuadrupedReferenceFrames referenceFrames;
   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame comFrame;
   private final QuadrantDependentList<MovingReferenceFrame> soleFrames;
   private final QuadrantDependentList<RigidBody> footRigidBody;
   private final RigidBody pelvisRigidBody;

   // estimates
   private final QuadrantDependentList<YoFrameOrientation> yoSoleOrientationEstimate;
   private final QuadrantDependentList<YoFramePoint> yoSolePositionEstimate;
   private final QuadrantDependentList<YoFrameVector> yoSoleAngularVelocityEstimate;
   private final QuadrantDependentList<YoFrameVector> yoSoleLinearVelocityEstimate;
   private final YoFrameOrientation yoBodyOrientationEstimate;
   private final YoFramePoint yoBodyPositionEstimate;
   private final YoFrameVector yoBodyAngularVelocityEstimate;
   private final YoFrameVector yoBodyLinearVelocityEstimate;
   private final YoFramePoint yoComPositionEstimate;
   private final YoFrameVector yoComVelocityEstimate;

   // solvers
   private final QuadrupedSoleForceEstimator soleForceEstimator;
   private final CenterOfMassJacobian comJacobian;
   private final Twist twistStorage;

   private final YoVariableRegistry registry = new YoVariableRegistry("taskSpaceEstimator");

   public QuadrupedTaskSpaceEstimator(FullQuadrupedRobotModel fullRobotModel, QuadrupedReferenceFrames referenceFrames, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry graphicsListRegistry)
   {
      this.referenceFrames = referenceFrames;
      comFrame = referenceFrames.getCenterOfMassFrame();
      bodyFrame = referenceFrames.getBodyFrame();
      soleFrames = referenceFrames.getFootReferenceFrames();
      pelvisRigidBody = fullRobotModel.getRootBody();
      footRigidBody = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         footRigidBody.set(robotQuadrant, fullRobotModel.getFoot(robotQuadrant));
      }

      // estimates
      yoSoleOrientationEstimate = new QuadrantDependentList<>();
      yoSolePositionEstimate = new QuadrantDependentList<>();
      yoSoleAngularVelocityEstimate = new QuadrantDependentList<>();
      yoSoleLinearVelocityEstimate = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseName();
         yoSoleOrientationEstimate.set(robotQuadrant, new YoFrameOrientation(prefix + "SoleOrientationEstimate", worldFrame, registry));
         yoSolePositionEstimate.set(robotQuadrant, new YoFramePoint(prefix + "SolePositionEstimate", worldFrame, registry));
         yoSoleAngularVelocityEstimate.set(robotQuadrant, new YoFrameVector(prefix + "SoleAngularVelocityEstimate", worldFrame, registry));
         yoSoleLinearVelocityEstimate.set(robotQuadrant, new YoFrameVector(prefix + "SoleLinearVelocityEstimate", worldFrame, registry));
      }
      yoBodyOrientationEstimate = new YoFrameOrientation("comPositionEstimate", worldFrame, registry);
      yoBodyPositionEstimate = new YoFramePoint("bodyPositionEstimate", worldFrame, registry);
      yoBodyAngularVelocityEstimate = new YoFrameVector("bodyAngularVelocityEstimate", worldFrame, registry);
      yoBodyLinearVelocityEstimate = new YoFrameVector("bodyLinearVelocityEstimate", worldFrame, registry);
      yoComPositionEstimate = new YoFramePoint("comPositionEstimate", worldFrame, registry);
      yoComVelocityEstimate = new YoFrameVector("comVelocityEstimate", worldFrame, registry);

      // graphics
      if (graphicsListRegistry != null)
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            String prefix = robotQuadrant.getCamelCaseName();
            YoFramePoint yoSolePosition = yoSolePositionEstimate.get(robotQuadrant);
            YoGraphicPosition yoSolePositionGraphic = new YoGraphicPosition(prefix + "SolePositionEstimate", yoSolePosition, 0.01, YoAppearance.Black());
            graphicsListRegistry.registerArtifact(prefix + "SolePositionEstimate", yoSolePositionGraphic.createArtifact());
         }
      }

      // solvers
      soleForceEstimator = new QuadrupedSoleForceEstimator(fullRobotModel, referenceFrames, registry);
      comJacobian = new CenterOfMassJacobian(fullRobotModel.getElevator());
      twistStorage = new Twist();

      parentRegistry.addChild(registry);
   }

   public void compute(QuadrupedTaskSpaceEstimates estimates)
   {
      // update solvers
      referenceFrames.updateFrames();
      soleForceEstimator.compute();
      comJacobian.compute();

      // compute sole poses, twists, and forces
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         footRigidBody.get(robotQuadrant).getBodyFixedFrame().getTwistOfFrame(twistStorage);
         twistStorage.changeFrame(soleFrames.get(robotQuadrant));
         twistStorage.getAngularPart(estimates.getSoleAngularVelocity().get(robotQuadrant));
         twistStorage.getLinearPart(estimates.getSoleLinearVelocity().get(robotQuadrant));
         estimates.getSoleOrientation().get(robotQuadrant).setToZero(soleFrames.get(robotQuadrant));
         estimates.getSolePosition(robotQuadrant).setToZero(soleFrames.get(robotQuadrant));
         estimates.getSoleVirtualForce().get(robotQuadrant).setIncludingFrame(soleForceEstimator.getSoleVirtualForce(robotQuadrant));
         estimates.getSoleContactForce().get(robotQuadrant).setIncludingFrame(soleForceEstimator.getSoleContactForce(robotQuadrant));
      }

      // compute body pose and twist
      pelvisRigidBody.getBodyFixedFrame().getTwistOfFrame(twistStorage);
      twistStorage.changeFrame(bodyFrame);
      twistStorage.getAngularPart(estimates.getBodyAngularVelocity());
      twistStorage.getLinearPart(estimates.getBodyLinearVelocity());
      estimates.getBodyOrientation().setToZero(bodyFrame);
      estimates.getBodyPosition().setToZero(bodyFrame);

      // compute center of mass position and velocity
      estimates.getComPosition().setToZero(comFrame);
      comJacobian.getCenterOfMassVelocity(estimates.getComVelocity());
      estimates.getComPosition().changeFrame(worldFrame);
      estimates.getComVelocity().changeFrame(worldFrame);

      // update yovariables
      yoBodyOrientationEstimate.setAndMatchFrame(estimates.getBodyOrientation());
      yoBodyPositionEstimate.setMatchingFrame(estimates.getBodyPosition());
      yoBodyAngularVelocityEstimate.setMatchingFrame(estimates.getBodyAngularVelocity());
      yoBodyLinearVelocityEstimate.setMatchingFrame(estimates.getBodyLinearVelocity());
      yoComPositionEstimate.setMatchingFrame(estimates.getComPosition());
      yoComVelocityEstimate.setMatchingFrame(estimates.getComVelocity());
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         yoSoleOrientationEstimate.get(robotQuadrant).setAndMatchFrame(estimates.getSoleOrientation().get(robotQuadrant));
         yoSolePositionEstimate.get(robotQuadrant).setMatchingFrame(estimates.getSolePosition(robotQuadrant));
         yoSoleAngularVelocityEstimate.get(robotQuadrant).setMatchingFrame(estimates.getSoleAngularVelocity().get(robotQuadrant));
         yoSoleLinearVelocityEstimate.get(robotQuadrant).setMatchingFrame(estimates.getSoleLinearVelocity().get(robotQuadrant));
      }
   }
}
