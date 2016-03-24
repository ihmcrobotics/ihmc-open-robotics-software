package us.ihmc.aware.controller.force.taskSpaceController;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class QuadrupedTaskSpaceEstimator
{
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
   private final CenterOfMassJacobian comJacobian;
   private final TwistCalculator twistCalculator;
   private final Twist twistStorage;

   private final YoVariableRegistry registry = new YoVariableRegistry("taskSpaceEstimator");

   public QuadrupedTaskSpaceEstimator(SDFFullRobotModel fullRobotModel, QuadrupedReferenceFrames referenceFrames, QuadrupedJointNameMap jointNameMap, YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
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
         String jointBeforeFootName = jointNameMap.getJointBeforeFootName(robotQuadrant);
         OneDoFJoint jointBeforeFoot = fullRobotModel.getOneDoFJointByName(jointBeforeFootName);
         footRigidBody.set(robotQuadrant, jointBeforeFoot.getSuccessor());
      }

      // estimates
      yoSoleOrientationEstimate = new QuadrantDependentList<>();
      yoSolePositionEstimate = new QuadrantDependentList<>();
      yoSoleAngularVelocityEstimate = new QuadrantDependentList<>();
      yoSoleLinearVelocityEstimate = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();
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

      // solvers
      comJacobian = new CenterOfMassJacobian(fullRobotModel.getElevator());
      twistCalculator = new TwistCalculator(worldFrame, fullRobotModel.getElevator());
      twistStorage = new Twist();

      parentRegistry.addChild(registry);
   }

   public void compute(QuadrupedTaskSpaceEstimates estimates)
   {
      // update solvers
      referenceFrames.updateFrames();
      twistCalculator.compute();
      comJacobian.compute();

      // compute sole poses and twists
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         twistCalculator.getTwistOfBody(twistStorage, footRigidBody.get(robotQuadrant));
         twistStorage.changeFrame(soleFrame.get(robotQuadrant));
         twistStorage.getAngularPart(estimates.getSoleAngularVelocity().get(robotQuadrant));
         twistStorage.getLinearPart(estimates.getSoleLinearVelocity().get(robotQuadrant));
         estimates.getSoleOrientation().get(robotQuadrant).setToZero(soleFrame.get(robotQuadrant));
         estimates.getSolePosition().get(robotQuadrant).setToZero(soleFrame.get(robotQuadrant));
      }

      // compute body pose and twist
      twistCalculator.getTwistOfBody(twistStorage, pelvisRigidBody);
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
