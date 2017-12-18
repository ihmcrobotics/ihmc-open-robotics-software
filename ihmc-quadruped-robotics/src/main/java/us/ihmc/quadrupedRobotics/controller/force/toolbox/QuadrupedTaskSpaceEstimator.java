package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;

public class QuadrupedTaskSpaceEstimator
{
   public static class Estimates
   {
      private final FrameQuaternion bodyOrientation = new FrameQuaternion();
      private final FramePoint3D bodyPosition = new FramePoint3D();
      private final FrameVector3D bodyLinearVelocity = new FrameVector3D();
      private final FrameVector3D bodyAngularVelocity = new FrameVector3D();
      private final FramePoint3D comPosition = new FramePoint3D();
      private final FrameVector3D comVelocity = new FrameVector3D();
      private final QuadrantDependentList<FrameQuaternion> soleOrientation = new QuadrantDependentList<>();
      private final QuadrantDependentList<FramePoint3D> solePosition = new QuadrantDependentList<>();
      private final QuadrantDependentList<FrameVector3D> soleAngularVelocity = new QuadrantDependentList<>();
      private final QuadrantDependentList<FrameVector3D> soleLinearVelocity = new QuadrantDependentList<>();
      private final QuadrantDependentList<FrameVector3D> soleVirtualForce = new QuadrantDependentList<>();
      private final QuadrantDependentList<FrameVector3D> soleContactForce = new QuadrantDependentList<>();

      public Estimates()
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            soleOrientation.set(robotQuadrant, new FrameQuaternion());
            solePosition.set(robotQuadrant, new FramePoint3D());
            soleAngularVelocity.set(robotQuadrant, new FrameVector3D());
            soleLinearVelocity.set(robotQuadrant, new FrameVector3D());
            soleVirtualForce.set(robotQuadrant, new FrameVector3D());
            soleContactForce.set(robotQuadrant, new FrameVector3D());
         }
      }

      public void set(Estimates other)
      {
         this.bodyOrientation.setIncludingFrame(other.bodyOrientation);
         this.bodyPosition.setIncludingFrame(other.bodyPosition);
         this.bodyLinearVelocity.setIncludingFrame(other.bodyLinearVelocity);
         this.bodyAngularVelocity.setIncludingFrame(other.bodyAngularVelocity);
         this.comPosition.setIncludingFrame(other.comPosition);
         this.comVelocity.setIncludingFrame(other.comVelocity);
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            this.soleOrientation.get(robotQuadrant).setIncludingFrame(other.soleOrientation.get(robotQuadrant));
            this.solePosition.get(robotQuadrant).setIncludingFrame(other.solePosition.get(robotQuadrant));
            this.soleAngularVelocity.get(robotQuadrant).setIncludingFrame(other.soleAngularVelocity.get(robotQuadrant));
            this.soleLinearVelocity.get(robotQuadrant).setIncludingFrame(other.soleLinearVelocity.get(robotQuadrant));
            this.soleVirtualForce.get(robotQuadrant).setIncludingFrame(other.soleVirtualForce.get(robotQuadrant));
            this.soleContactForce.get(robotQuadrant).setIncludingFrame(other.soleContactForce.get(robotQuadrant));
         }
      }

      public FrameQuaternion getBodyOrientation()
      {
         return bodyOrientation;
      }

      public FramePoint3D getBodyPosition()
      {
         return bodyPosition;
      }

      public FrameVector3D getBodyLinearVelocity()
      {
         return bodyLinearVelocity;
      }

      public FrameVector3D getBodyAngularVelocity()
      {
         return bodyAngularVelocity;
      }

      public FramePoint3D getComPosition()
      {
         return comPosition;
      }

      public FrameVector3D getComVelocity()
      {
         return comVelocity;
      }

      public FrameQuaternion getSoleOrientation(RobotQuadrant robotQuadrant)
      {
         return soleOrientation.get(robotQuadrant);
      }

      public FramePoint3D getSolePosition(RobotQuadrant robotQuadrant)
      {
         return solePosition.get(robotQuadrant);
      }

      public FrameVector3D getSoleAngularVelocity(RobotQuadrant robotQuadrant)
      {
         return soleAngularVelocity.get(robotQuadrant);
      }

      public FrameVector3D getSoleLinearVelocity(RobotQuadrant robotQuadrant)
      {
         return soleLinearVelocity.get(robotQuadrant);
      }

      public FrameVector3D getSoleVirtualForce(RobotQuadrant robotQuadrant)
      {
         return soleVirtualForce.get(robotQuadrant);
      }

      public FrameVector3D getSoleContactForce(RobotQuadrant robotQuadrant)
      {
         return soleContactForce.get(robotQuadrant);
      }

      public QuadrantDependentList<FrameQuaternion> getSoleOrientation()
      {
         return soleOrientation;
      }

      public QuadrantDependentList<FramePoint3D> getSolePosition()
      {
         return solePosition;
      }

      public QuadrantDependentList<FrameVector3D> getSoleAngularVelocity()
      {
         return soleAngularVelocity;
      }

      public QuadrantDependentList<FrameVector3D> getSoleLinearVelocity()
      {
         return soleLinearVelocity;
      }

      public QuadrantDependentList<FrameVector3D> getSoleVirtualForce()
      {
         return soleVirtualForce;
      }

      public QuadrantDependentList<FrameVector3D> getSoleContactForce()
      {
         return soleContactForce;
      }
   }

   private final QuadrupedReferenceFrames referenceFrames;
   private final ReferenceFrame worldFrame;
   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame comFrame;
   private final QuadrantDependentList<ReferenceFrame> soleFrame;
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
      comFrame = referenceFrames.getCenterOfMassZUpFrame();
      bodyFrame = referenceFrames.getBodyFrame();
      worldFrame = referenceFrames.getWorldFrame();
      soleFrame = referenceFrames.getFootReferenceFrames();
      pelvisRigidBody = fullRobotModel.getPelvis();
      footRigidBody = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         OneDoFJoint jointBeforeFoot = fullRobotModel.getOneDoFJointBeforeFoot(robotQuadrant);
         footRigidBody.set(robotQuadrant, jointBeforeFoot.getSuccessor());
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
      yoBodyOrientationEstimate = new YoFrameOrientation("bodyOrientationEstimate", worldFrame, registry);
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

   public void compute(Estimates estimates)
   {
      // update solvers
      referenceFrames.updateFrames();
      soleForceEstimator.compute();
      comJacobian.compute();

      // compute sole poses, twists, and forces
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         footRigidBody.get(robotQuadrant).getBodyFixedFrame().getTwistOfFrame(twistStorage);
         twistStorage.changeFrame(soleFrame.get(robotQuadrant));
         twistStorage.getAngularPart(estimates.getSoleAngularVelocity().get(robotQuadrant));
         twistStorage.getLinearPart(estimates.getSoleLinearVelocity().get(robotQuadrant));
         estimates.getSoleOrientation().get(robotQuadrant).setToZero(soleFrame.get(robotQuadrant));
         estimates.getSolePosition().get(robotQuadrant).setToZero(soleFrame.get(robotQuadrant));
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
      yoBodyPositionEstimate.setAndMatchFrame(estimates.getBodyPosition());
      yoBodyAngularVelocityEstimate.setAndMatchFrame(estimates.getBodyAngularVelocity());
      yoBodyLinearVelocityEstimate.setAndMatchFrame(estimates.getBodyLinearVelocity());
      yoComPositionEstimate.setAndMatchFrame(estimates.getComPosition());
      yoComVelocityEstimate.setAndMatchFrame(estimates.getComVelocity());
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         yoSoleOrientationEstimate.get(robotQuadrant).setAndMatchFrame(estimates.getSoleOrientation().get(robotQuadrant));
         yoSolePositionEstimate.get(robotQuadrant).setAndMatchFrame(estimates.getSolePosition().get(robotQuadrant));
         yoSoleAngularVelocityEstimate.get(robotQuadrant).setAndMatchFrame(estimates.getSoleAngularVelocity().get(robotQuadrant));
         yoSoleLinearVelocityEstimate.get(robotQuadrant).setAndMatchFrame(estimates.getSoleLinearVelocity().get(robotQuadrant));
      }
   }
}
