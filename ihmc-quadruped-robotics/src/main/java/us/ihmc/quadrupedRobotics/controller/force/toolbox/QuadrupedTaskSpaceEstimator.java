package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;
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
   private final QuadrantDependentList<YoFrameYawPitchRoll> yoSoleOrientationEstimate;
   private final QuadrantDependentList<YoFramePoint3D> yoSolePositionEstimate;
   private final QuadrantDependentList<YoFrameVector3D> yoSoleAngularVelocityEstimate;
   private final QuadrantDependentList<YoFrameVector3D> yoSoleLinearVelocityEstimate;
   private final YoFrameYawPitchRoll yoBodyOrientationEstimate;
   private final YoFramePoint3D yoBodyPositionEstimate;
   private final YoFrameVector3D yoBodyAngularVelocityEstimate;
   private final YoFrameVector3D yoBodyLinearVelocityEstimate;
   private final YoFramePoint3D yoComPositionEstimate;
   private final YoFrameVector3D yoComVelocityEstimate;

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
         yoSoleOrientationEstimate.set(robotQuadrant, new YoFrameYawPitchRoll(prefix + "SoleOrientationEstimate", worldFrame, registry));
         yoSolePositionEstimate.set(robotQuadrant, new YoFramePoint3D(prefix + "SolePositionEstimate", worldFrame, registry));
         yoSoleAngularVelocityEstimate.set(robotQuadrant, new YoFrameVector3D(prefix + "SoleAngularVelocityEstimate", worldFrame, registry));
         yoSoleLinearVelocityEstimate.set(robotQuadrant, new YoFrameVector3D(prefix + "SoleLinearVelocityEstimate", worldFrame, registry));
      }
      yoBodyOrientationEstimate = new YoFrameYawPitchRoll("comPositionEstimate", worldFrame, registry);
      yoBodyPositionEstimate = new YoFramePoint3D("bodyPositionEstimate", worldFrame, registry);
      yoBodyAngularVelocityEstimate = new YoFrameVector3D("bodyAngularVelocityEstimate", worldFrame, registry);
      yoBodyLinearVelocityEstimate = new YoFrameVector3D("bodyLinearVelocityEstimate", worldFrame, registry);
      yoComPositionEstimate = new YoFramePoint3D("comPositionEstimate", worldFrame, registry);
      yoComVelocityEstimate = new YoFrameVector3D("comVelocityEstimate", worldFrame, registry);

      // graphics
      if (graphicsListRegistry != null)
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            String prefix = robotQuadrant.getCamelCaseName();
            YoFramePoint3D yoSolePosition = yoSolePositionEstimate.get(robotQuadrant);
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

   public CenterOfMassJacobian getComJacobian()
   {
      return comJacobian;
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
      yoBodyOrientationEstimate.setMatchingFrame(estimates.getBodyOrientation());
      yoBodyPositionEstimate.setMatchingFrame(estimates.getBodyPosition());
      yoBodyAngularVelocityEstimate.setMatchingFrame(estimates.getBodyAngularVelocity());
      yoBodyLinearVelocityEstimate.setMatchingFrame(estimates.getBodyLinearVelocity());
      yoComPositionEstimate.setMatchingFrame(estimates.getComPosition());
      yoComVelocityEstimate.setMatchingFrame(estimates.getComVelocity());
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         yoSoleOrientationEstimate.get(robotQuadrant).setMatchingFrame(estimates.getSoleOrientation().get(robotQuadrant));
         yoSolePositionEstimate.get(robotQuadrant).setMatchingFrame(estimates.getSolePosition(robotQuadrant));
         yoSoleAngularVelocityEstimate.get(robotQuadrant).setMatchingFrame(estimates.getSoleAngularVelocity().get(robotQuadrant));
         yoSoleLinearVelocityEstimate.get(robotQuadrant).setMatchingFrame(estimates.getSoleLinearVelocity().get(robotQuadrant));
      }
   }
}
