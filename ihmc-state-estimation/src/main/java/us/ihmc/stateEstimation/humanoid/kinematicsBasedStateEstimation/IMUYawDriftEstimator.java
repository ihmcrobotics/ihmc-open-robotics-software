package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.commons.FormattingTools;

public class IMUYawDriftEstimator implements YawDriftProvider
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final int numberOfFeet;
   private final List<RigidBody> allFeet;
   private final List<RigidBody> trustedFeet = new ArrayList<>();
   private final Map<RigidBody, FootSwitchInterface> footSwitches;
   private final Map<RigidBody, ReferenceFrame> footSoleFrames = new LinkedHashMap<>();

   private final YoInteger numberOfFeetTrusted = new YoInteger("numberOfFeetTrustedIMUDrift", registry);

   private final Map<RigidBody, GlitchFilteredYoBoolean> areFeetTrusted = new LinkedHashMap<>();
   private final YoDouble delayBeforeTrustingFoot = new YoDouble("delayBeforeTrustingFootIMUDrift", registry);

   private final YoFramePoint referenceAverageFootPosition = new YoFramePoint("referenceAverageFootPositionIMUDrift", worldFrame, registry);
   private final Map<RigidBody, YoFramePoint> referenceFootPositions = new LinkedHashMap<>();
   private final YoFramePoint currentAverageFootPosition = new YoFramePoint("currentAverageFootPositionIMUDrift", worldFrame, registry);
   private final Map<RigidBody, YoFramePoint> currentFootPositions = new LinkedHashMap<>();

   private final YoDouble footLinearVelocityThreshold = new YoDouble("footLinearVelocityThreshold", registry);
   private final Map<RigidBody, YoDouble> currentFootLinearVelocities = new LinkedHashMap<>();

   private final Map<RigidBody, YoDouble> estimatedYawDriftPerFoot = new LinkedHashMap<>();

   private final YoDouble estimatedYawDrift = new YoDouble("estimatedRawYawDrift", registry);
   private final YoDouble estimatedYawDriftPrevious = new YoDouble("estimatedYawDriftPrevious", registry);
   private final YoDouble yawDriftAlphaFilter = new YoDouble("yawDriftAlphaFilter", registry);
   private final YoDouble estimatedFilteredYawDrift = new YoDouble("estimatedFilteredYawDrift", registry);

   private final YoDouble previouslyEstimatedYawDrift = new YoDouble("previouslyEstimatedYawDrift", registry);
   private final YoDouble totalEstimatedYawDrift = new YoDouble("totalEstimatedYawDrift", registry);
   private final YoDouble yawDriftRateAlphaFilter = new YoDouble("yawDriftRateAlphaFilter", registry);
   private final AlphaFilteredYoVariable estimatedYawDriftRate = new AlphaFilteredYoVariable("estimatedYawDriftRate", registry, yawDriftRateAlphaFilter);

   private final YoBoolean enableCompensation = new YoBoolean("enableIMUDriftYawCompensation", registry);
   private final YoBoolean integrateDriftRate = new YoBoolean("integrateDriftRate", registry);

   private final FramePoint3D footPosition = new FramePoint3D();
   private final FramePoint3D averagePosition = new FramePoint3D();
   private final FrameVector3D referenceAverageToFootPosition = new FrameVector3D();
   private final FrameVector3D currentAverageToFootPosition = new FrameVector3D();
   private final Twist footTwist = new Twist();
   private final FrameVector3D footLinearVelocity = new FrameVector3D();

   private final double estimatorDT;

   public IMUYawDriftEstimator(FullInverseDynamicsStructure inverseDynamicsStructure, Map<RigidBody, FootSwitchInterface> footSwitches,
         Map<RigidBody, ? extends ContactablePlaneBody> feet, final double estimatorDT, YoVariableRegistry parentRegistry)
   {
      this.estimatorDT = estimatorDT;
      this.footSwitches = footSwitches;
      allFeet = new ArrayList<>(footSwitches.keySet());
      numberOfFeet = allFeet.size();

      for (int i = 0; i < numberOfFeet; i++)
      {
         RigidBody foot = allFeet.get(i);
         String footNamePascalCase = FormattingTools.underscoredToCamelCase(foot.getName(), true);

         int windowSize = (int) (delayBeforeTrustingFoot.getDoubleValue() / estimatorDT);
         GlitchFilteredYoBoolean isFootTrusted = new GlitchFilteredYoBoolean("is" + footNamePascalCase + "TrustedIMUDrift", registry,
               windowSize);
         areFeetTrusted.put(foot, isFootTrusted);

         ReferenceFrame soleFrame = feet.get(foot).getSoleFrame();
         footSoleFrames.put(foot, soleFrame);
      }

      for (int i = 0; i < numberOfFeet; i++)
      {
         RigidBody foot = allFeet.get(i);
         String footNameCamelCase = FormattingTools.underscoredToCamelCase(foot.getName(), false);
         YoFramePoint referenceFootPosition = new YoFramePoint(footNameCamelCase + "IMUDriftReference", worldFrame, registry);
         referenceFootPositions.put(foot, referenceFootPosition);

      }

      for (int i = 0; i < numberOfFeet; i++)
      {
         RigidBody foot = allFeet.get(i);
         String footNameCamelCase = FormattingTools.underscoredToCamelCase(foot.getName(), false);
         YoFramePoint currentFootPosition = new YoFramePoint(footNameCamelCase + "IMUDriftCurrentPosition", worldFrame, registry);
         currentFootPositions.put(foot, currentFootPosition);

      }

      for (int i = 0; i < numberOfFeet; i++)
      {
         RigidBody foot = allFeet.get(i);
         String footNameCamelCase = FormattingTools.underscoredToCamelCase(foot.getName(), false);
         YoDouble currentFootLinearVelocity = new YoDouble(footNameCamelCase + "IMUDriftCurrentLinearVelocityMag", registry);
         currentFootLinearVelocities.put(foot, currentFootLinearVelocity);

      }

      for (int i = 0; i < numberOfFeet; i++)
      {
         RigidBody foot = allFeet.get(i);
         String footNameCamelCase = FormattingTools.underscoredToCamelCase(foot.getName(), false);
         YoDouble estimatedYawDriftFoot = new YoDouble(footNameCamelCase + "EstimatedYawDrift", registry);
         estimatedYawDriftPerFoot.put(foot, estimatedYawDriftFoot);
      }

      delayBeforeTrustingFoot.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            int windowSize = (int) (delayBeforeTrustingFoot.getDoubleValue() / estimatorDT);
            for (int i = 0; i < allFeet.size(); i++)
               areFeetTrusted.get(allFeet.get(i)).setWindowSize(windowSize);
         }
      });

      parentRegistry.addChild(registry);

      estimatedYawDriftRate.update(0.0);

//      configureModuleParameters(true, true, 0.5, 0.8, 0.04, 1.5e-3);
   }

   public void configureModuleParameters(StateEstimatorParameters stateEstimatorParameters)
   {
      this.enableCompensation.set(stateEstimatorParameters.enableIMUYawDriftCompensation());
      this.integrateDriftRate.set(stateEstimatorParameters.integrateEstimatedIMUYawDriftRate());
      this.delayBeforeTrustingFoot.set(stateEstimatorParameters.getIMUYawDriftEstimatorDelayBeforeTrustingFoot());
      this.footLinearVelocityThreshold.set(stateEstimatorParameters.getIMUYawDriftFootLinearVelocityThreshold());

      double driftFilterFrequency = stateEstimatorParameters.getIMUYawDriftFilterFreqInHertz();
      double driftRateFilterFrequency = stateEstimatorParameters.getIMUYawDriftRateFilterFreqInHertz();
      this.yawDriftAlphaFilter.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(driftFilterFrequency, this.estimatorDT));
      this.yawDriftRateAlphaFilter.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(driftRateFilterFrequency, this.estimatorDT));
   }

   public void configureModuleParameters(boolean enableCompensation, boolean integrateRate, double delayBeforeTrustingFoot, double footLinearVelocityThreshold, double driftFilterFrequency, double driftRateFilterFrequency)
   {
      this.enableCompensation.set(enableCompensation);
      this.integrateDriftRate.set(integrateRate);
      this.delayBeforeTrustingFoot.set(delayBeforeTrustingFoot);
      this.yawDriftAlphaFilter.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(driftFilterFrequency, this.estimatorDT));
      this.yawDriftRateAlphaFilter.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(driftRateFilterFrequency, this.estimatorDT));
      this.footLinearVelocityThreshold.set(footLinearVelocityThreshold);
   }

   public void update()
   {
      updateFootLinearVelocities();
      updateTrustedFeetAndReferences();
      updateCurrents();
      estimateYawDriftPerFoot();
      updateYawDrift();

      if (enableCompensation.getBooleanValue())
         totalEstimatedYawDrift.set(previouslyEstimatedYawDrift.getDoubleValue() + estimatedFilteredYawDrift.getDoubleValue());
   }

   private void updateFootLinearVelocities()
   {
      for (int i = 0; i < numberOfFeet; i++)
      {
         RigidBody foot = allFeet.get(i);
         foot.getBodyFixedFrame().getTwistOfFrame(footTwist);
         footTwist.getLinearPart(footLinearVelocity);
         currentFootLinearVelocities.get(foot).set(footLinearVelocity.length());
      }
   }

   private void updateTrustedFeetAndReferences()
   {
      trustedFeet.clear();

      int newNumberOfFeetTrusted = 0;

      for (int i = 0; i < numberOfFeet; i++)
      {
         RigidBody foot = allFeet.get(i);
         GlitchFilteredYoBoolean isFootTrusted = areFeetTrusted.get(foot);

         boolean hasFootHitGround = footSwitches.get(foot).hasFootHitGround();
         boolean isFootStatic = currentFootLinearVelocities.get(foot).getDoubleValue() < footLinearVelocityThreshold.getDoubleValue();

         if (enableCompensation.getBooleanValue() && hasFootHitGround && isFootStatic)
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
            RigidBody foot = allFeet.get(i);
            footPosition.setToZero(footSoleFrames.get(foot));
            referenceFootPositions.get(foot).setAndMatchFrame(footPosition);
         }

         referenceAverageFootPosition.setToZero();
         for (int i = 0; i < trustedFeet.size(); i++)
         {
            RigidBody foot = trustedFeet.get(i);
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
            RigidBody foot = allFeet.get(i);
            footPosition.setToZero(footSoleFrames.get(foot));
            currentFootPositions.get(foot).setAndMatchFrame(footPosition);
         }

         currentAverageFootPosition.setToZero();
         for (int i = 0; i < numberOfFeet; i++)
         {
            RigidBody foot = allFeet.get(i);
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
         RigidBody foot = allFeet.get(i);
         referenceAverageFootPosition.getFrameTupleIncludingFrame(averagePosition);
         referenceFootPositions.get(foot).getFrameTupleIncludingFrame(footPosition);
         referenceAverageToFootPosition.setToZero(worldFrame);
         referenceAverageToFootPosition.sub(footPosition, averagePosition);

         currentAverageFootPosition.getFrameTupleIncludingFrame(averagePosition);
         currentFootPositions.get(foot).getFrameTupleIncludingFrame(footPosition);
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

         if (integrateDriftRate.getBooleanValue())
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
      estimatedFilteredYawDrift.set(AngleTools.trimAngleMinusPiToPi(yawDriftAlphaFilter.getDoubleValue() * angleDifference + estimatedYawDrift.getDoubleValue()));

      if (!estimatedYawDriftPrevious.isNaN())
      {
         double rate = AngleTools.computeAngleDifferenceMinusPiToPi(estimatedYawDrift.getDoubleValue(), estimatedYawDriftPrevious.getDoubleValue()) / estimatorDT;
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
