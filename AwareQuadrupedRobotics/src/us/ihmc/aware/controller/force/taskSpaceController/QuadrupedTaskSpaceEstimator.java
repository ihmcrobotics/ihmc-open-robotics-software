package us.ihmc.aware.controller.force.taskSpaceController;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedPhysicalProperties;
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
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class QuadrupedTaskSpaceEstimator
{
   private final double gravity = 9.81;
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
   private final DoubleYoVariable yoLipNaturalFrequency;

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
      yoLipNaturalFrequency = new DoubleYoVariable("lipNaturalFrequency", registry);

      // solvers
      comJacobian = new CenterOfMassJacobian(fullRobotModel.getElevator());
      twistCalculator = new TwistCalculator(worldFrame, fullRobotModel.getElevator());
      twistStorage = new Twist();

      parentRegistry.addChild(registry);
   }

   public QuadrupedReferenceFrames getReferenceFrames()
   {
      return referenceFrames;
   }

   public void compute(QuadrupedTaskSpaceEstimates outputEstimates, QuadrupedTaskSpaceCommands inputCommands, QuadrupedTaskSpaceEstimatorSettings settings)
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
         twistStorage.getAngularPart(outputEstimates.getSoleAngularVelocity().get(robotQuadrant));
         twistStorage.getLinearPart(outputEstimates.getSoleLinearVelocity().get(robotQuadrant));
         outputEstimates.getSoleOrientation().get(robotQuadrant).setToZero(soleFrame.get(robotQuadrant));
         outputEstimates.getSolePosition().get(robotQuadrant).setToZero(soleFrame.get(robotQuadrant));
      }

      // compute body pose and twist
      twistCalculator.getTwistOfBody(twistStorage, pelvisRigidBody);
      twistStorage.changeFrame(bodyFrame);
      twistStorage.getAngularPart(outputEstimates.getBodyAngularVelocity());
      twistStorage.getLinearPart(outputEstimates.getBodyLinearVelocity());
      outputEstimates.getBodyOrientation().setToZero(bodyFrame);
      outputEstimates.getBodyPosition().setToZero(bodyFrame);

      // compute center of mass position and velocity
      outputEstimates.getComPosition().setToZero(comFrame);
      comJacobian.getCenterOfMassVelocity(outputEstimates.getComVelocity());

      // compute divergent component of motion and instantaneous capture point
      double naturalFrequency = settings.getLipNaturalFrequency();
      outputEstimates.setLipNaturalFrequency(naturalFrequency);
      outputEstimates.getComPosition().changeFrame(worldFrame);
      outputEstimates.getComVelocity().changeFrame(worldFrame);
      outputEstimates.getDcmPosition().changeFrame(worldFrame);
      outputEstimates.getDcmPosition().setX(outputEstimates.getComPosition().getX() + outputEstimates.getComVelocity().getX() / naturalFrequency);
      outputEstimates.getDcmPosition().setY(outputEstimates.getComPosition().getY() + outputEstimates.getComVelocity().getY() / naturalFrequency);
      outputEstimates.getDcmPosition().setZ(outputEstimates.getComPosition().getZ() + outputEstimates.getComVelocity().getZ() / naturalFrequency);
      outputEstimates.getIcpPosition().setIncludingFrame(outputEstimates.getDcmPosition());
      outputEstimates.getIcpPosition().sub(0, 0, gravity / (naturalFrequency * naturalFrequency));

      // update variables
      yoBodyOrientationEstimate.setAndMatchFrame(outputEstimates.getBodyOrientation());
      yoBodyAngularVelocityEstimate.setAndMatchFrame(outputEstimates.getBodyAngularVelocity());
      yoBodyLinearVelocityEstimate.setAndMatchFrame(outputEstimates.getBodyLinearVelocity());
      yoComPositionEstimate.setAndMatchFrame(outputEstimates.getComPosition());
      yoComVelocityEstimate.setAndMatchFrame(outputEstimates.getComVelocity());
      yoDcmPositionEstimate.setAndMatchFrame(outputEstimates.getDcmPosition());
      yoIcpPositionEstimate.setAndMatchFrame(outputEstimates.getIcpPosition());
      yoLipNaturalFrequency.set(outputEstimates.getLipNaturalFrequency());
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         yoSoleOrientationEstimate.get(robotQuadrant).setAndMatchFrame(outputEstimates.getSoleOrientation().get(robotQuadrant));
         yoSolePositionEstimate.get(robotQuadrant).setAndMatchFrame(outputEstimates.getSolePosition().get(robotQuadrant));
         yoSoleAngularVelocityEstimate.get(robotQuadrant).setAndMatchFrame(outputEstimates.getSoleAngularVelocity().get(robotQuadrant));
         yoSoleLinearVelocityEstimate.get(robotQuadrant).setAndMatchFrame(outputEstimates.getSoleLinearVelocity().get(robotQuadrant));
      }
   }
}
