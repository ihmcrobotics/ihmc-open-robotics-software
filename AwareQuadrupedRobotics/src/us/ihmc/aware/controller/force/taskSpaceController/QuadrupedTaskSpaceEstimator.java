package us.ihmc.aware.controller.force.taskSpaceController;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.*;

public class QuadrupedTaskSpaceEstimator
{
   private final double gravity = 9.81;
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
   private final YoFrameConvexPolygon2d yoSupportPolygonEstimate;
   private final YoFramePoint yoSupportCentroidEstimate;
   private final YoFrameOrientation yoSupportOrientationEstimate;
   private final YoFrameOrientation yoBodyOrientationEstimate;
   private final YoFramePoint yoBodyPositionEstimate;
   private final YoFrameVector yoBodyAngularVelocityEstimate;
   private final YoFrameVector yoBodyLinearVelocityEstimate;
   private final YoFramePoint yoComPositionEstimate;
   private final YoFrameVector yoComVelocityEstimate;
   private final YoFramePoint yoDcmPositionEstimate;
   private final YoFramePoint yoIcpPositionEstimate;
   private final DoubleYoVariable yoComHeightEstimate;

   // solvers
   private final CenterOfMassJacobian comJacobian;
   private final TwistCalculator twistCalculator;
   private final Twist twistStorage;

   private final YoVariableRegistry registry = new YoVariableRegistry("quadrupedTaskSpaceEstimator");

   public QuadrupedTaskSpaceEstimator(SDFFullRobotModel fullRobotModel, QuadrupedReferenceFrames referenceFrames, QuadrupedJointNameMap jointNameMap, YoVariableRegistry parentRegistry)
   {
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
      yoSupportPolygonEstimate = new YoFrameConvexPolygon2d("supportPolygonEstimate", "", worldFrame, 4, registry);
      yoSupportCentroidEstimate = new YoFramePoint("supportCentroidEstimate", worldFrame, registry);
      yoSupportOrientationEstimate = new YoFrameOrientation("supportOrientationEstimate", worldFrame, registry);
      yoBodyOrientationEstimate = new YoFrameOrientation("bodyOrientationEstimate", worldFrame, registry);
      yoBodyPositionEstimate = new YoFramePoint("bodyPositionEstimate", worldFrame, registry);
      yoBodyAngularVelocityEstimate = new YoFrameVector("bodyAngularVelocityEstimate", worldFrame, registry);
      yoBodyLinearVelocityEstimate = new YoFrameVector("bodyLinearVelocityEstimate", worldFrame, registry);
      yoComPositionEstimate = new YoFramePoint("comPositionEstimate", worldFrame, registry);
      yoComVelocityEstimate = new YoFrameVector("comVelocityEstimate", worldFrame, registry);
      yoDcmPositionEstimate = new YoFramePoint("dcmPositionEstimate", worldFrame, registry);
      yoIcpPositionEstimate = new YoFramePoint("icpPositionEstimate", worldFrame, registry);
      yoComHeightEstimate = new DoubleYoVariable("comHeightEstimate", registry);

      // solvers
      comJacobian = new CenterOfMassJacobian(fullRobotModel.getElevator());
      twistCalculator = new TwistCalculator(worldFrame, fullRobotModel.getElevator());
      twistStorage = new Twist();

      parentRegistry.addChild(registry);
   }

   public void compute(QuadrupedTaskSpaceEstimates estimates, QuadrupedTaskSpaceEstimatorParameters estimatorParameters)
   {
      // update solvers
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

      // compute support polygon
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         estimates.getSolePosition().get(robotQuadrant).changeFrame(estimates.getSupportPolygon().getReferenceFrame());
         estimates.getSupportPolygon().setFootstep(robotQuadrant, estimates.getSolePosition().get(robotQuadrant));
      }

      // compute support frame (centroid and nominal orientation)
      estimates.getSupportCentroid().changeFrame(estimates.getSupportPolygon().getReferenceFrame());
      estimates.getSupportOrientation().changeFrame(estimates.getSupportPolygon().getReferenceFrame());
      estimates.getSupportPolygon().getCentroid2d(estimates.getSupportCentroid());
      estimates.getSupportOrientation().setYawPitchRoll(estimates.getSupportPolygon().getNominalYaw(), 0, 0);

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

      // compute divergent component of motion and instantaneous capture point
      estimates.getComPosition().changeFrame(worldFrame);
      estimates.getComVelocity().changeFrame(worldFrame);
      estimates.getDcmPosition().changeFrame(worldFrame);
      double omega = estimatorParameters.getDcmNaturalFrequency();
      double comHeightOffset = gravity / (omega * omega);
      estimates.getDcmPosition().setX(estimates.getComPosition().getX() + estimates.getComVelocity().getX() / omega);
      estimates.getDcmPosition().setY(estimates.getComPosition().getY() + estimates.getComVelocity().getY() / omega);
      estimates.getDcmPosition().setZ(estimates.getComPosition().getZ() + estimates.getComVelocity().getZ() / omega);
      estimates.getIcpPosition().setIncludingFrame(estimates.getDcmPosition());
      estimates.getIcpPosition().add(0, 0, -comHeightOffset);

      // compute center of mass height
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         estimates.getSolePosition().get(robotQuadrant).changeFrame(worldFrame);
      }
      double minFrontFootHeight = Math.min(estimates.getSolePosition().get(RobotQuadrant.FRONT_LEFT).getZ(), estimates.getSolePosition().get(RobotQuadrant.FRONT_RIGHT).getZ());
      double minHindFootHeight = Math.min(estimates.getSolePosition().get(RobotQuadrant.HIND_LEFT).getZ(), estimates.getSolePosition().get(RobotQuadrant.HIND_RIGHT).getZ());
      estimates.getComPosition().changeFrame(worldFrame);
      estimates.setComHeight(estimates.getComPosition().getZ() - ((minFrontFootHeight + minHindFootHeight) / 2.0));

      // update variables
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         yoSoleOrientationEstimate.get(robotQuadrant).setAndMatchFrame(estimates.getSoleOrientation().get(robotQuadrant));
         yoSolePositionEstimate.get(robotQuadrant).setAndMatchFrame(estimates.getSolePosition().get(robotQuadrant));
         yoSoleAngularVelocityEstimate.get(robotQuadrant).setAndMatchFrame(estimates.getSoleAngularVelocity().get(robotQuadrant));
         yoSoleLinearVelocityEstimate.get(robotQuadrant).setAndMatchFrame(estimates.getSoleLinearVelocity().get(robotQuadrant));
      }
      estimates.getSupportPolygon().packYoFrameConvexPolygon2d(yoSupportPolygonEstimate);
      yoSupportCentroidEstimate.setAndMatchFrame(estimates.getSupportCentroid());
      yoSupportOrientationEstimate.setAndMatchFrame(estimates.getSupportOrientation());
      yoBodyOrientationEstimate.setAndMatchFrame(estimates.getBodyOrientation());
      yoBodyAngularVelocityEstimate.setAndMatchFrame(estimates.getBodyAngularVelocity());
      yoBodyLinearVelocityEstimate.setAndMatchFrame(estimates.getBodyLinearVelocity());
      yoComPositionEstimate.setAndMatchFrame(estimates.getComPosition());
      yoComVelocityEstimate.setAndMatchFrame(estimates.getComVelocity());
      yoDcmPositionEstimate.setAndMatchFrame(estimates.getDcmPosition());
      yoIcpPositionEstimate.setAndMatchFrame(estimates.getIcpPosition());
      yoComHeightEstimate.set(estimates.getComHeight());
   }
}
