package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commons.FormattingTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoInteger;

public class IMUYawDriftEstimator implements YawDriftProvider
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final int numberOfFeet;
   private final List<RigidBodyBasics> allFeet;
   private final List<RigidBodyBasics> trustedFeet = new ArrayList<>();
   private final Map<RigidBodyBasics, FootSwitchInterface> footSwitches;
   private final Map<RigidBodyBasics, ReferenceFrame> footSoleFrames = new LinkedHashMap<>();

   private final YoInteger numberOfFeetTrusted = new YoInteger("numberOfFeetTrustedIMUDrift", registry);

   private final Map<RigidBodyBasics, GlitchFilteredYoBoolean> areFeetTrusted = new LinkedHashMap<>();
   private final DoubleProvider delayBeforeTrustingFoot;

   private final YoFramePoint3D referenceAverageFootPosition = new YoFramePoint3D("referenceAverageFootPositionIMUDrift", worldFrame, registry);
   private final Map<RigidBodyBasics, YoFramePoint3D> referenceFootPositions = new LinkedHashMap<>();
   private final YoFramePoint3D currentAverageFootPosition = new YoFramePoint3D("currentAverageFootPositionIMUDrift", worldFrame, registry);
   private final Map<RigidBodyBasics, YoFramePoint3D> currentFootPositions = new LinkedHashMap<>();

   private final DoubleProvider footLinearVelocityThreshold;
   private final Map<RigidBodyBasics, YoDouble> currentFootLinearVelocities = new LinkedHashMap<>();

   private final Map<RigidBodyBasics, YoDouble> estimatedYawDriftPerFoot = new LinkedHashMap<>();

   private final YoDouble estimatedYawDrift = new YoDouble("estimatedRawYawDrift", registry);
   private final YoDouble estimatedYawDriftPrevious = new YoDouble("estimatedYawDriftPrevious", registry);
   private final DoubleProvider yawDriftBreakFrequency;
   private final YoDouble estimatedFilteredYawDrift = new YoDouble("estimatedFilteredYawDrift", registry);

   private final YoDouble previouslyEstimatedYawDrift = new YoDouble("previouslyEstimatedYawDrift", registry);
   private final YoDouble totalEstimatedYawDrift = new YoDouble("totalEstimatedYawDrift", registry);
   private final DoubleProvider yawDriftRateBreakFrequency;
   private final AlphaFilteredYoVariable estimatedYawDriftRate;

   private final BooleanProvider enableCompensation;
   private final BooleanProvider integrateDriftRate;

   /**
    * Affects how this estimator starts trusting the feet for estimating the yaw drift.
    * <p>
    * When set to {@code false} (default and recommended behavior), the foot switches are used to
    * determine whether a foot is firmly on the ground. When set to {@code true}, this estimator
    * trust that all feet are firmly on the ground as soon as the controller reports
    * {@link RobotMotionStatus#STANDING}.
    * </p>
    */
   private final BooleanParameter estimateWhenControllerIsStanding = new BooleanParameter("estimateYawDriftWhenControllerIsStanding", registry, false);
   private final RobotMotionStatusHolder robotMotionStatusFromController;

   private final FramePoint3D footPosition = new FramePoint3D();
   private final FramePoint3D averagePosition = new FramePoint3D();
   private final FrameVector3D referenceAverageToFootPosition = new FrameVector3D();
   private final FrameVector3D currentAverageToFootPosition = new FrameVector3D();

   private final double estimatorDT;

   public IMUYawDriftEstimator(FullInverseDynamicsStructure inverseDynamicsStructure, Map<RigidBodyBasics, FootSwitchInterface> footSwitches,
                               Map<RigidBodyBasics, ? extends ContactablePlaneBody> feet, RobotMotionStatusHolder robotMotionStatusFromController,
                               StateEstimatorParameters stateEstimatorParameters, YoVariableRegistry parentRegistry)
   {
      this.robotMotionStatusFromController = robotMotionStatusFromController;
      this.estimatorDT = stateEstimatorParameters.getEstimatorDT();
      this.footSwitches = footSwitches;
      allFeet = new ArrayList<>(footSwitches.keySet());
      numberOfFeet = allFeet.size();

      enableCompensation = new BooleanParameter("enableIMUDriftYawCompensation", registry, stateEstimatorParameters.enableIMUYawDriftCompensation());
      integrateDriftRate = new BooleanParameter("integrateDriftRate", registry, stateEstimatorParameters.integrateEstimatedIMUYawDriftRate());
      delayBeforeTrustingFoot = new DoubleParameter("delayBeforeTrustingFootIMUDrift", registry,
                                                    stateEstimatorParameters.getIMUYawDriftEstimatorDelayBeforeTrustingFoot());
      footLinearVelocityThreshold = new DoubleParameter("footLinearVelocityThreshold", registry,
                                                        stateEstimatorParameters.getIMUYawDriftFootLinearVelocityThreshold());
      yawDriftBreakFrequency = new DoubleParameter("yawDriftBreakFrequency", registry, stateEstimatorParameters.getIMUYawDriftFilterFreqInHertz());
      yawDriftRateBreakFrequency = new DoubleParameter("yawDriftRateBreakFrequency", registry, stateEstimatorParameters.getIMUYawDriftRateFilterFreqInHertz());

      DoubleProvider alphaYawDrift = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(yawDriftRateBreakFrequency.getValue(), estimatorDT);
      estimatedYawDriftRate = new AlphaFilteredYoVariable("estimatedYawDriftRate", registry, alphaYawDrift);

      for (int i = 0; i < numberOfFeet; i++)
      {
         RigidBodyBasics foot = allFeet.get(i);
         String footNamePascalCase = FormattingTools.underscoredToCamelCase(foot.getName(), true);

         GlitchFilteredYoBoolean isFootTrusted = new GlitchFilteredYoBoolean("is" + footNamePascalCase + "TrustedIMUDrift", registry, 0);
         areFeetTrusted.put(foot, isFootTrusted);

         ReferenceFrame soleFrame = feet.get(foot).getSoleFrame();
         footSoleFrames.put(foot, soleFrame);
      }

      for (int i = 0; i < numberOfFeet; i++)
      {
         RigidBodyBasics foot = allFeet.get(i);
         String footNameCamelCase = FormattingTools.underscoredToCamelCase(foot.getName(), false);
         YoFramePoint3D referenceFootPosition = new YoFramePoint3D(footNameCamelCase + "IMUDriftReference", worldFrame, registry);
         referenceFootPositions.put(foot, referenceFootPosition);

      }

      for (int i = 0; i < numberOfFeet; i++)
      {
         RigidBodyBasics foot = allFeet.get(i);
         String footNameCamelCase = FormattingTools.underscoredToCamelCase(foot.getName(), false);
         YoFramePoint3D currentFootPosition = new YoFramePoint3D(footNameCamelCase + "IMUDriftCurrentPosition", worldFrame, registry);
         currentFootPositions.put(foot, currentFootPosition);

      }

      for (int i = 0; i < numberOfFeet; i++)
      {
         RigidBodyBasics foot = allFeet.get(i);
         String footNameCamelCase = FormattingTools.underscoredToCamelCase(foot.getName(), false);
         YoDouble currentFootLinearVelocity = new YoDouble(footNameCamelCase + "IMUDriftCurrentLinearVelocityMag", registry);
         currentFootLinearVelocities.put(foot, currentFootLinearVelocity);

      }

      for (int i = 0; i < numberOfFeet; i++)
      {
         RigidBodyBasics foot = allFeet.get(i);
         String footNameCamelCase = FormattingTools.underscoredToCamelCase(foot.getName(), false);
         YoDouble estimatedYawDriftFoot = new YoDouble(footNameCamelCase + "EstimatedYawDrift", registry);
         estimatedYawDriftPerFoot.put(foot, estimatedYawDriftFoot);
      }

      parentRegistry.addChild(registry);

      estimatedYawDriftRate.update(0.0);
   }

   public void initialize()
   {
      estimatedYawDriftRate.reset();
      estimatedYawDriftRate.update(0.0);

      for (int i = 0; i < numberOfFeet; i++)
      {
         RigidBodyBasics foot = allFeet.get(i);
         areFeetTrusted.get(foot).set(false);
         areFeetTrusted.get(foot).update(false);
      }
   }

   public void update()
   {
      updateFootLinearVelocities();
      updateTrustedFeetAndReferences();
      updateCurrents();
      estimateYawDriftPerFoot();
      updateYawDrift();

      if (enableCompensation.getValue())
         totalEstimatedYawDrift.set(previouslyEstimatedYawDrift.getDoubleValue() + estimatedFilteredYawDrift.getDoubleValue());
   }

   private void updateFootLinearVelocities()
   {
      for (int i = 0; i < numberOfFeet; i++)
      {
         RigidBodyBasics foot = allFeet.get(i);
         TwistReadOnly footTwist = foot.getBodyFixedFrame().getTwistOfFrame();
         currentFootLinearVelocities.get(foot).set(footTwist.getLinearPart().length());
      }
   }

   private void updateTrustedFeetAndReferences()
   {
      trustedFeet.clear();

      int newNumberOfFeetTrusted = 0;

      int windowSize = (int) (delayBeforeTrustingFoot.getValue() / estimatorDT);
      for (int i = 0; i < allFeet.size(); i++)
         areFeetTrusted.get(allFeet.get(i)).setWindowSize(windowSize);

      boolean isStanding = robotMotionStatusFromController.getCurrentRobotMotionStatus() == RobotMotionStatus.STANDING;

      for (int i = 0; i < numberOfFeet; i++)
      {
         RigidBodyBasics foot = allFeet.get(i);
         GlitchFilteredYoBoolean isFootTrusted = areFeetTrusted.get(foot);

         boolean hasFootHitGround;

         if (estimateWhenControllerIsStanding.getValue())
            hasFootHitGround = isStanding;
         else
            hasFootHitGround = isStanding && footSwitches.get(foot).hasFootHitGround();

         boolean isFootStatic = currentFootLinearVelocities.get(foot).getDoubleValue() < footLinearVelocityThreshold.getValue();

         if (enableCompensation.getValue() && hasFootHitGround && isFootStatic)
            isFootTrusted.update(true);
         else
            isFootTrusted.set(false);

         if (isFootTrusted.getBooleanValue())
         {
            newNumberOfFeetTrusted++;
            trustedFeet.add(foot);
         }
      }

      // Just started trusting all the feet.
      // Initialize the reference points.
      if (numberOfFeetTrusted.getIntegerValue() != newNumberOfFeetTrusted && newNumberOfFeetTrusted == numberOfFeet)
      {
         for (int i = 0; i < numberOfFeet; i++)
         {
            RigidBodyBasics foot = allFeet.get(i);
            footPosition.setToZero(footSoleFrames.get(foot));
            referenceFootPositions.get(foot).setMatchingFrame(footPosition);
         }

         referenceAverageFootPosition.setToZero();
         for (int i = 0; i < trustedFeet.size(); i++)
         {
            RigidBodyBasics foot = trustedFeet.get(i);
            referenceAverageFootPosition.add(referenceFootPositions.get(foot));
         }
         referenceAverageFootPosition.scale(1.0 / trustedFeet.size());
      }
      // Not trusting all the feet.
      // Reseting all the reference points.
      else if (newNumberOfFeetTrusted < numberOfFeet)
      {
         for (int i = 0; i < numberOfFeet; i++)
            referenceFootPositions.get(allFeet.get(i)).setToNaN();
         referenceAverageFootPosition.setToNaN();

         // Just stopped trusting all the feet.
         // Save the estimated drift for the next stance.
         if (numberOfFeetTrusted.getIntegerValue() == numberOfFeet)
         {
            previouslyEstimatedYawDrift.set(totalEstimatedYawDrift.getDoubleValue());
         }
      }

      numberOfFeetTrusted.set(newNumberOfFeetTrusted);
   }

   private void updateCurrents()
   {
      if (numberOfFeetTrusted.getIntegerValue() == numberOfFeet)
      {
         for (int i = 0; i < numberOfFeet; i++)
         {
            RigidBodyBasics foot = allFeet.get(i);
            footPosition.setToZero(footSoleFrames.get(foot));
            currentFootPositions.get(foot).setMatchingFrame(footPosition);
         }

         currentAverageFootPosition.setToZero();
         for (int i = 0; i < numberOfFeet; i++)
         {
            RigidBodyBasics foot = allFeet.get(i);
            currentAverageFootPosition.add(currentFootPositions.get(foot));
         }
         currentAverageFootPosition.scale(1.0 / numberOfFeet);
      }
      else
      {
         for (int i = 0; i < numberOfFeet; i++)
            currentFootPositions.get(allFeet.get(i)).setToNaN();
         currentAverageFootPosition.setToNaN();
      }
   }

   private void estimateYawDriftPerFoot()
   {
      if (numberOfFeetTrusted.getIntegerValue() != numberOfFeet)
      {
         for (int i = 0; i < numberOfFeet; i++)
            estimatedYawDriftPerFoot.get(allFeet.get(i)).set(Double.NaN);
         return;
      }

      for (int i = 0; i < numberOfFeet; i++)
      {
         RigidBodyBasics foot = allFeet.get(i);
         averagePosition.setIncludingFrame(referenceAverageFootPosition);
         footPosition.setIncludingFrame(referenceFootPositions.get(foot));
         referenceAverageToFootPosition.setToZero(worldFrame);
         referenceAverageToFootPosition.sub(footPosition, averagePosition);

         averagePosition.setIncludingFrame(currentAverageFootPosition);
         footPosition.setIncludingFrame(currentFootPositions.get(foot));
         currentAverageToFootPosition.setToZero(worldFrame);
         currentAverageToFootPosition.sub(footPosition, averagePosition);

         double refenceAngle = Math.atan2(referenceAverageToFootPosition.getY(), referenceAverageToFootPosition.getX());
         double currentAngle = Math.atan2(currentAverageToFootPosition.getY(), currentAverageToFootPosition.getX());
         estimatedYawDriftPerFoot.get(foot).set(AngleTools.computeAngleDifferenceMinusPiToPi(currentAngle, refenceAngle));
      }
   }

   private void updateYawDrift()
   {
      if (numberOfFeetTrusted.getIntegerValue() != numberOfFeet)
      {
         estimatedYawDrift.set(Double.NaN);
         estimatedYawDriftPrevious.set(Double.NaN);
         estimatedFilteredYawDrift.set(0.0);

         if (integrateDriftRate.getValue())
         {
            previouslyEstimatedYawDrift.add(estimatedYawDriftRate.getDoubleValue() * estimatorDT);
         }

         return;
      }

      double yawDrift = 0.0;

      for (int i = 0; i < numberOfFeet; i++)
         yawDrift += estimatedYawDriftPerFoot.get(allFeet.get(i)).getDoubleValue();

      yawDrift /= numberOfFeet;
      estimatedYawDrift.set(yawDrift);
      double angleDifference = AngleTools.computeAngleDifferenceMinusPiToPi(estimatedFilteredYawDrift.getDoubleValue(), estimatedYawDrift.getDoubleValue());
      double alphaYawDrift = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(yawDriftBreakFrequency.getValue(), estimatorDT);
      estimatedFilteredYawDrift.set(AngleTools.trimAngleMinusPiToPi(alphaYawDrift * angleDifference + estimatedYawDrift.getDoubleValue()));

      if (!estimatedYawDriftPrevious.isNaN())
      {
         double rate = AngleTools.computeAngleDifferenceMinusPiToPi(estimatedYawDrift.getDoubleValue(), estimatedYawDriftPrevious.getDoubleValue())
               / estimatorDT;
         estimatedYawDriftRate.update(rate);
      }
      estimatedYawDriftPrevious.set(estimatedYawDrift.getDoubleValue());
   }

   @Override
   public double getYawBiasInWorldFrame()
   {
      return totalEstimatedYawDrift.getDoubleValue();
   }
}
