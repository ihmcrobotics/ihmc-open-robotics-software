package us.ihmc.commonWalkingControlModules.kinematics;

import javax.vecmath.Tuple3d;

import us.ihmc.sensorProcessing.stateEstimation.BodyPositionEstimator;
import us.ihmc.sensorProcessing.stateEstimation.LegToTrustForVelocityReadOnly;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFramePoint;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.Twist;


public class BodyPositionEstimatorThroughStanceLeg implements BodyPositionEstimator
{
   private final String name;
   private final YoVariableRegistry registry;
   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();
   private final RobotSide robotSide;
   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame ankleZUpFrame;
   private final LegToTrustForVelocityReadOnly legToTrustForVelocity;
   private final FootTwistCalculator footTwistCalculator;
   private final YoFramePoint anklePositionFix;
   private final DoubleYoVariable alphaAnklePositionFix;
   private final AlphaFilteredYoFramePoint filteredAnklePositionFix;
   private final YoFramePoint bodyPositionThroughStanceLeg;
   private final DoubleYoVariable defaultCovariance;
   private final DoubleYoVariable currentCovariance;
   private final DoubleYoVariable footSlippingAngularVelocityLimit;
   private final DoubleYoVariable footSlippingLinearVelocityLimit;
   private final DoubleYoVariable footAngularVelocityMagnitude;
   private final DoubleYoVariable footLinearVelocityMagnitude;
   private final double ankleHeight;

   public BodyPositionEstimatorThroughStanceLeg(RobotSide robotSide, FootTwistCalculator footTwistCalculator, LegToTrustForVelocityReadOnly legToTrustForVelocity, ReferenceFrame bodyFrame, ReferenceFrame ankleZUpFrame, double defaultCovariance, double ankleHeight, double footSlippingAngularVelocityLimit, double footSlippingLinearVelocityLimit, double anklePositionFixBreakFrequencyHertz, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.name = robotSide + getClass().getSimpleName();
      this.registry = new YoVariableRegistry(name);
      this.robotSide = robotSide;
      this.bodyFrame = bodyFrame;
      this.ankleZUpFrame = ankleZUpFrame;
      this.legToTrustForVelocity = legToTrustForVelocity;
      this.footTwistCalculator = footTwistCalculator;
      this.anklePositionFix = new YoFramePoint("anklePositionFix", "", world, registry);
      this.alphaAnklePositionFix = new DoubleYoVariable("alphaPositionFix", registry);
      this.filteredAnklePositionFix = AlphaFilteredYoFramePoint.createAlphaFilteredYoFramePoint("filteredAnklePositionFix", "", registry, alphaAnklePositionFix, anklePositionFix);
      this.bodyPositionThroughStanceLeg = new YoFramePoint("bodyPositionThrough" + robotSide.getCamelCaseNameForMiddleOfExpression() + "StanceLeg", "", world, registry);
      this.defaultCovariance = new DoubleYoVariable("bodyPositionThrough" + robotSide.getCamelCaseNameForMiddleOfExpression() + "StanceLegDefaultCovariance", registry);
      this.currentCovariance = new DoubleYoVariable(robotSide.getCamelCaseNameForStartOfExpression() + "PositionThroughStanceLegCurrentCovariance", registry);
      this.footSlippingAngularVelocityLimit = new DoubleYoVariable("footSlippingAngularVelocityLimit", registry);
      this.footSlippingLinearVelocityLimit = new DoubleYoVariable("footSlippingLinearVelocityLimit", registry);
      this.footAngularVelocityMagnitude = new DoubleYoVariable(robotSide.getCamelCaseNameForStartOfExpression() + "FootAngularVelocityMagnitude", registry);
      this.footLinearVelocityMagnitude = new DoubleYoVariable(robotSide.getCamelCaseNameForStartOfExpression() + "FootLinearVelocityMagnitude", registry);
      alphaAnklePositionFix.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequency(anklePositionFixBreakFrequencyHertz, controlDT));
      
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
      double covariance;
      if (this.legToTrustForVelocity.isLegTrustedForVelocity(robotSide) && isFootStationary())
      {
         updateAnklePositionInWorld();
         covariance = defaultCovariance.getDoubleValue();
      }
      else
         covariance = Double.POSITIVE_INFINITY;
      
      double epsilon = 1e-9;
      if (covariance < (currentCovariance.getDoubleValue() - epsilon))
         resetAnklePositionInWorld();
      currentCovariance.set(covariance);
      
      tempBodyPosition.setToZero(bodyFrame);
      tempBodyPosition.changeFrame(ankleZUpFrame);
      tempBodyPositionVector.setIncludingFrame(tempBodyPosition);
      tempBodyPositionVector.changeFrame(world);
      bodyPositionThroughStanceLeg.set(filteredAnklePositionFix);
      bodyPositionThroughStanceLeg.add(tempBodyPositionVector);
   }

   public void packBodyPosition(FramePoint bodyPositionToPack)
   {
      bodyPositionThroughStanceLeg.getFrameTuple(bodyPositionToPack);
   }
   
   private void updateAnklePositionInWorld()
   {
      FramePoint ankle = new FramePoint(ankleZUpFrame);
      ankle.changeFrame(world);
      ankle.setZ(ankleHeight);
      anklePositionFix.set(ankle);
      filteredAnklePositionFix.update();
   }
   
   private void resetAnklePositionInWorld()
   {
      filteredAnklePositionFix.reset();
      filteredAnklePositionFix.update();
   }

   public void packCovariance(Tuple3d covarianceToPack)
   {
      covarianceToPack.set(currentCovariance.getDoubleValue(), currentCovariance.getDoubleValue(), currentCovariance.getDoubleValue());
   }
   
   private final FrameVector tempAngularPart = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector tempLinearPart = new FrameVector(ReferenceFrame.getWorldFrame());
   private boolean isFootStationary()
   {
      if (footTwistCalculator == null)
      {
         return true;
      }
      
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