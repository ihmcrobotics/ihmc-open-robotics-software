package us.ihmc.commonWalkingControlModules.kinematics;

import javax.vecmath.Tuple3d;

import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.LegToTrustForVelocityReadOnly;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

public class BodyPositionEstimatorThroughStanceLeg implements BodyPositionEstimator
{
   private final String name;
   private final YoVariableRegistry registry;
   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();
   private final RobotSide robotSide;
   private final CommonWalkingReferenceFrames referenceFrames;
   private final LegToTrustForVelocityReadOnly legToTrustForVelocity;
   private final YoFramePoint anklePositionFix;
   private final FramePoint bodyPosition = new FramePoint(world);
   private final DoubleYoVariable defaultCovariance;
   private final double ankleHeight;

   public BodyPositionEstimatorThroughStanceLeg(RobotSide robotSide, LegToTrustForVelocityReadOnly legToTrustForVelocity, CommonWalkingReferenceFrames referenceFrames, double defaultCovariance, double ankleHeight, YoVariableRegistry parentRegistry)
   {
      this.name = robotSide + getClass().getSimpleName();
      this.registry = new YoVariableRegistry(name);
      this.robotSide = robotSide;
      this.referenceFrames = referenceFrames;
      this.legToTrustForVelocity = legToTrustForVelocity;
      this.anklePositionFix = new YoFramePoint("anklePositionFix", "", world, registry);
      this.defaultCovariance = new DoubleYoVariable(robotSide.getCamelCaseNameForStartOfExpression() + "PositionThroughStanceLegCovariance", registry);
      this.defaultCovariance.set(defaultCovariance);
      this.ankleHeight = ankleHeight;
      parentRegistry.addChild(registry);
   }
   
   private final FrameVector tempBodyPositionVector = new FrameVector(world);
   public void estimateBodyPosition()
   {
      bodyPosition.setToZero(referenceFrames.getIMUFrame());
      bodyPosition.changeFrame(referenceFrames.getAnkleZUpFrame(robotSide));
      tempBodyPositionVector.setAndChangeFrame(bodyPosition);
      tempBodyPositionVector.changeFrame(world);
      bodyPosition.setToZero(world);
      anklePositionFix.getFramePoint(bodyPosition);
      bodyPosition.add(tempBodyPositionVector);
   }
   
   public void packBodyPosition(FramePoint bodyPositionToPack)
   {
      bodyPositionToPack.set(bodyPosition);
   }
   
   private void fixAnklePositionInWorld()
   {
      FramePoint ankle = new FramePoint(referenceFrames.getAnkleZUpFrame(robotSide));
      ankle.changeFrame(world);
      ankle.setZ(ankleHeight);
      anklePositionFix.set(ankle);
   }

   public void packCovariance(Tuple3d covarianceToPack)
   {
      // TODO: also use angular velocity of foot with respect to ground. Create FootAngularVelocityCalculator; do only once
      RobotSide legToTrustForVelocity = this.legToTrustForVelocity.getLegToTrustForVelocity();
      double covariance;
      if (legToTrustForVelocity == null)
      {
         covariance = defaultCovariance.getDoubleValue();
      }
      else
      {
         boolean trustingLegForVelocity = legToTrustForVelocity == robotSide;
         covariance = trustingLegForVelocity ? defaultCovariance.getDoubleValue() : Double.POSITIVE_INFINITY;
      }

      covarianceToPack.set(covariance, covariance, covariance);
   }

   public void configureAfterEstimation()
   {
      fixAnklePositionInWorld();
   }
}
