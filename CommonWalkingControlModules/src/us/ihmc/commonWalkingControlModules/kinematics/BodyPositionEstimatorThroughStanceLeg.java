package us.ihmc.commonWalkingControlModules.kinematics;

import javax.vecmath.Tuple3d;

import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.LegToTrustForVelocityReadOnly;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.Twist;

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
   private final FootTwistCalculator footTwistCalculator;
   private final YoFramePoint anklePositionFix;
   private final YoFramePoint bodyPositionThroughStanceLeg;
   private final DoubleYoVariable defaultCovariance;
   private final DoubleYoVariable currentCovariance;
   private final DoubleYoVariable footSlippingAngularVelocityLimit;
   private final DoubleYoVariable footSlippingLinearVelocityLimit;
   private final DoubleYoVariable footAngularVelocityMagnitude;
   private final DoubleYoVariable footLinearVelocityMagnitude;
   private final double ankleHeight;

   public BodyPositionEstimatorThroughStanceLeg(RobotSide robotSide, ProcessedSensorsInterface processedSensors, LegToTrustForVelocityReadOnly legToTrustForVelocity, CommonWalkingReferenceFrames referenceFrames, double defaultCovariance, double ankleHeight, double footSlippingAngularVelocityLimit, double footSlippingLinearVelocityLimit, YoVariableRegistry parentRegistry)
   {
      this.name = robotSide + getClass().getSimpleName();
      this.registry = new YoVariableRegistry(name);
      this.robotSide = robotSide;
      this.referenceFrames = referenceFrames;
      this.legToTrustForVelocity = legToTrustForVelocity;
      this.footTwistCalculator = new FootTwistCalculator(robotSide, processedSensors);
      this.anklePositionFix = new YoFramePoint("anklePositionFix", "", world, registry);
      this.bodyPositionThroughStanceLeg = new YoFramePoint("bodyPositionThrough" + robotSide.getCamelCaseNameForMiddleOfExpression() + "StanceLeg", "", world, registry);
      this.defaultCovariance = new DoubleYoVariable("bodyPositionThrough" + robotSide.getCamelCaseNameForMiddleOfExpression() + "StanceLegDefaultCovariance", registry);
      this.currentCovariance = new DoubleYoVariable(robotSide.getCamelCaseNameForStartOfExpression() + "PositionThroughStanceLegCurrentCovariance", registry);
      this.footSlippingAngularVelocityLimit = new DoubleYoVariable("footSlippingAngularVelocityLimit", registry);
      this.footSlippingLinearVelocityLimit = new DoubleYoVariable("footSlippingLinearVelocityLimit", registry);
      this.footAngularVelocityMagnitude = new DoubleYoVariable(robotSide.getCamelCaseNameForStartOfExpression() + "FootAngularVelocityMagnitude", registry);
      this.footLinearVelocityMagnitude = new DoubleYoVariable(robotSide.getCamelCaseNameForStartOfExpression() + "FootLinearVelocityMagnitude", registry);
      
      this.defaultCovariance.set(defaultCovariance);
      this.footSlippingAngularVelocityLimit.set(footSlippingAngularVelocityLimit);
      this.footSlippingLinearVelocityLimit.set(footSlippingLinearVelocityLimit);

      this.ankleHeight = ankleHeight;
      parentRegistry.addChild(registry);
   }
   
   private final FramePoint tempBodyPosition = new FramePoint(world);
   private final FrameVector tempBodyPositionVector = new FrameVector(world);
   public void estimateBodyPosition()
   {     
      tempBodyPosition.setToZero(referenceFrames.getIMUFrame());
      tempBodyPosition.changeFrame(referenceFrames.getAnkleZUpFrame(robotSide));
      tempBodyPositionVector.setAndChangeFrame(tempBodyPosition);
      tempBodyPositionVector.changeFrame(world);
      bodyPositionThroughStanceLeg.set(anklePositionFix);
      bodyPositionThroughStanceLeg.add(tempBodyPositionVector);

      double covariance;
      if (this.legToTrustForVelocity.isLegTrustedForVelocity(robotSide) && isFootStationary())
         covariance = defaultCovariance.getDoubleValue();
      else
         covariance = Double.POSITIVE_INFINITY;

      if (covariance < currentCovariance.getDoubleValue())
         fixAnklePositionInWorld();
      currentCovariance.set(covariance);
   }
   
   public void packBodyPosition(FramePoint bodyPositionToPack)
   {
      bodyPositionThroughStanceLeg.getFramePoint(bodyPositionToPack);
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
      covarianceToPack.set(currentCovariance.getDoubleValue(), currentCovariance.getDoubleValue(), currentCovariance.getDoubleValue());
   }
   
   private final FrameVector tempAngularPart = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector tempLinearPart = new FrameVector(ReferenceFrame.getWorldFrame());
   private boolean isFootStationary()
   {
      Twist footTwist = footTwistCalculator.computeFootTwist();
      footTwist.packAngularPart(tempAngularPart);
      footTwist.packBodyOriginLinearPartInBaseFrame(tempLinearPart);
      
      footAngularVelocityMagnitude.set(tempAngularPart.length());
      footLinearVelocityMagnitude.set(tempLinearPart.length());
      
      boolean angularVelocityOK = footAngularVelocityMagnitude.getDoubleValue() < footSlippingAngularVelocityLimit.getDoubleValue();
      boolean linearVelocityOK = footLinearVelocityMagnitude.getDoubleValue() < footSlippingLinearVelocityLimit.getDoubleValue();
      return angularVelocityOK && linearVelocityOK;
   }
}
