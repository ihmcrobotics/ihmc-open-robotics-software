package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.FiniteDifferenceAngularVelocityYoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;

/**
 * IMUDriftCompensator when activated estimates the IMU drift on the yaw angle and correct the root joint orientation and angular velocity around the vertical axis.
 * @author Sylvain
 *
 */
public class IMUDriftCompensator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final BooleanYoVariable userForceIMUDriftCompensation = new BooleanYoVariable("userForceIMUDriftCompensation", registry);

   private final BooleanYoVariable isIMUDriftCompensationActivated = new BooleanYoVariable("isIMUDriftCompensationActivated", registry);
   private final BooleanYoVariable isIMUDriftYawRateEstimationActivated = new BooleanYoVariable("isIMUDriftYawRateEstimationActivated", registry);
   private final BooleanYoVariable isIMUDriftYawRateEstimated = new BooleanYoVariable("isIMUDriftYawRateEstimated", registry);
   private final BooleanYoVariable isIMUDriftFeetLoadedEnough = new BooleanYoVariable("isIMUDriftFeetLoadedEnough", registry);

   private final DoubleYoVariable alphaFilterIMUDrift = new DoubleYoVariable("alphaFilterIMUDrift", registry);
   private final DoubleYoVariable imuDriftYawRate = new DoubleYoVariable("estimatedIMUDriftYawRate", registry);
   private final AlphaFilteredYoVariable imuDriftYawRateFiltered = new AlphaFilteredYoVariable("estimatedIMUDriftYawRateFiltered", registry,
                                                                                               alphaFilterIMUDrift, imuDriftYawRate);
   private final DoubleYoVariable imuDriftYawAngle = new DoubleYoVariable("estimatedIMUDriftYawAngle", registry);

   private final DoubleYoVariable rootJointYawAngleCorrected = new DoubleYoVariable("rootJointYawAngleWithDriftCompensation", registry);
   private final DoubleYoVariable rootJointYawRateCorrected = new DoubleYoVariable("rootJointYawRateWithDriftCompensation", registry);

   private final Map<RigidBody, YoFrameQuaternion> footOrientationsInWorld = new LinkedHashMap<RigidBody, YoFrameQuaternion>();
   private final DoubleYoVariable alphaFilterFootAngularVelocity = new DoubleYoVariable("alphaFilterFootAngularVelocity", registry);
   private final Map<RigidBody, FiniteDifferenceAngularVelocityYoFrameVector> footAngularVelocitiesInWorld = new LinkedHashMap<RigidBody, FiniteDifferenceAngularVelocityYoFrameVector>();
   private final Map<RigidBody, AlphaFilteredYoVariable> footAngularVelocitiesInWorldFilteredX = new LinkedHashMap<RigidBody, AlphaFilteredYoVariable>();
   private final Map<RigidBody, AlphaFilteredYoVariable> footAngularVelocitiesInWorldFilteredY = new LinkedHashMap<RigidBody, AlphaFilteredYoVariable>();
   private final Map<RigidBody, AlphaFilteredYoVariable> footAngularVelocitiesInWorldFilteredZ = new LinkedHashMap<RigidBody, AlphaFilteredYoVariable>();
   private final YoFrameVector yoTotalFootAngularVelocityDifferencesFromAverage = new YoFrameVector("totalFootAngularVelocityDifferenceFromAverage", worldFrame, registry);
   private final YoFrameVector yoFootAngularVelocityAverage = new YoFrameVector("footAngularVelocityAverage", worldFrame, registry);
   private final YoFrameVector footAngularVelocityDifferenceThresholdToEstimateIMUDrift = new YoFrameVector("footAngularVelocityDifferenceThresholdToEstimateIMUDrift", worldFrame, registry);
   private final BooleanYoVariable areFootAngularVelocitiesCloseEnough = new BooleanYoVariable("areFootAngularVelocitiesCloseEnough", registry);

   private final List<RigidBody> feet = new ArrayList<>();
   private final Map<RigidBody, ReferenceFrame> footFrames;

   private final double estimatorDT;

   private final TwistCalculator twistCalculator;

   private final SixDoFJoint rootJoint;
   private final ReferenceFrame rootJointFrame;

   private final Map<RigidBody, FootSwitchInterface> footSwitches;
   private final DoubleYoVariable totalLoadPercentageOnFeet = new DoubleYoVariable("totalLoadPercentageOnFeet", registry);
   private final DoubleYoVariable loadPercentageOnFeetThresholdForIMUDrift = new DoubleYoVariable("loadPercentageOnFeetThresholdForIMUDrift", registry);

   // temporary variables
   private final FrameVector footAngularVelocityAverage = new FrameVector(worldFrame);
   private final Map<RigidBody, FrameOrientation> footOrientations = new LinkedHashMap<RigidBody, FrameOrientation>();

   public IMUDriftCompensator(Map<RigidBody, ReferenceFrame> footFrames, FullInverseDynamicsStructure inverseDynamicsStructure,
         Map<RigidBody, FootSwitchInterface> footSwitches, double estimatorDT, YoVariableRegistry parentRegistry)
   {
      this.rootJoint = inverseDynamicsStructure.getRootJoint();
      this.footFrames = footFrames;
      this.rootJointFrame = rootJoint.getFrameAfterJoint();
      this.estimatorDT = estimatorDT;
      this.twistCalculator = inverseDynamicsStructure.getTwistCalculator();
      this.footSwitches = footSwitches;
      loadPercentageOnFeetThresholdForIMUDrift.set(0.5);
      this.feet.addAll(footFrames.keySet());

      for (int i = 0; i < feet.size(); i++)
      {
         RigidBody foot = feet.get(i);
         String namePrefix = foot.getName();

         YoFrameQuaternion footOrientationInWorld = new YoFrameQuaternion(namePrefix + "FootOrientationInWorld", worldFrame, registry);
         footOrientationsInWorld.put(foot, footOrientationInWorld);
         
         AlphaFilteredYoVariable footAngularVelocityInWorldX = new AlphaFilteredYoVariable(namePrefix + "FootAngularVelocityInWorldFilteredX", registry, alphaFilterFootAngularVelocity);
         footAngularVelocitiesInWorldFilteredX.put(foot, footAngularVelocityInWorldX);

         AlphaFilteredYoVariable footAngularVelocityInWorldY = new AlphaFilteredYoVariable(namePrefix + "FootAngularVelocityInWorldFilteredY", registry, alphaFilterFootAngularVelocity);
         footAngularVelocitiesInWorldFilteredY.put(foot, footAngularVelocityInWorldY);

         AlphaFilteredYoVariable footAngularVelocityInWorldZ = new AlphaFilteredYoVariable(namePrefix + "FootAngularVelocityInWorldFilteredZ", registry, alphaFilterFootAngularVelocity);
         footAngularVelocitiesInWorldFilteredZ.put(foot, footAngularVelocityInWorldZ);
         
         footOrientations.put(foot, new FrameOrientation());

         FiniteDifferenceAngularVelocityYoFrameVector footAngularVelocityInWorld = new FiniteDifferenceAngularVelocityYoFrameVector(namePrefix + "FootAngularVelocitiesInWorld", footOrientationInWorld, estimatorDT, registry);
         footAngularVelocitiesInWorld.put(foot, footAngularVelocityInWorld);
      }

      isIMUDriftYawRateEstimated.set(false);
      imuDriftYawRate.set(0.0);
      imuDriftYawRateFiltered.reset();
      imuDriftYawRateFiltered.update();
      imuDriftYawAngle.set(0.0);
      rootJointYawAngleCorrected.set(0.0);

      parentRegistry.addChild(registry);
   }

   public void activateEstimation(boolean activate)
   {
      isIMUDriftYawRateEstimationActivated.set(activate);
   }

   public void activateCompensation(boolean activate)
   {
      isIMUDriftCompensationActivated.set(activate);
   }

   public void setAlphaIMUDrift(double alphaFilter)
   {
      alphaFilterIMUDrift.set(alphaFilter);
   }

   public void setAlphaFootAngularVelocity(double alphaFilter)
   {
      alphaFilterFootAngularVelocity.set(alphaFilter);
   }

   public void setFootAngularVelocityThreshold(double threshold)
   {
      footAngularVelocityDifferenceThresholdToEstimateIMUDrift.set(threshold, threshold, threshold);
   }

   public void initialize()
   {
      imuDriftYawRate.set(0.0);
      imuDriftYawRateFiltered.reset();
      resetFootAngularVelocitiesFiltered();
      updateFootOrientations();
      resetFootAngularVelocitiesFiltered();
      updateFootOrientations();

      boolean areFeetLoadedEnough = areFeetLoadedEnough();
      if (isIMUDriftYawRateEstimationActivated.getBooleanValue() && areFeetLoadedEnough)
      {
         isIMUDriftYawRateEstimated.set(true);
         estimateIMUDriftYaw(null);
      }
      else
      {
         isIMUDriftYawRateEstimated.set(false);
      }

      if (isIMUDriftCompensationActivated.getBooleanValue())
         compensateIMUDriftYaw();
   }

   public void updateAndCompensateDrift()
   {
      boolean areFeetLoadedEnough = areFeetLoadedEnough();
      if (!isIMUDriftYawRateEstimationActivated.getBooleanValue() || !areFeetLoadedEnough)
      {
         resetFootAngularVelocitiesFiltered();
         updateFootOrientations();
         resetFootAngularVelocitiesFiltered();
         isIMUDriftYawRateEstimated.set(false);
      }

      updateFootOrientations();

      if (isIMUDriftCompensationActivated.getBooleanValue())
         compensateIMUDriftYaw();
   }

   /**
    * Estimate the IMU yaw drift if the feet angular velocities are low enough.
    * @param trustedSide Refers to the foot to trust, set it to null when both feet are trusted.
    */
   public void esimtateDriftIfPossible(List<RigidBody> trustedFeet)
   {
      if (userForceIMUDriftCompensation.getBooleanValue())
      {
         isIMUDriftYawRateEstimated.set(true);
         estimateIMUDriftYaw(trustedFeet);

         return;
      }

      if (trustedFeet.size() < 2)
      {
         isIMUDriftYawRateEstimated.set(false);
         return;
      }

      if (isIMUDriftYawRateEstimationActivated.getBooleanValue() && areFeetLoadedEnough() && areFeetAngularVelocitiesClose(trustedFeet))
      {
         isIMUDriftYawRateEstimated.set(true);
         estimateIMUDriftYaw(trustedFeet);
      }
      else
      {
         isIMUDriftYawRateEstimated.set(false);
      }
   }

   private boolean areFeetLoadedEnough()
   {
      double totalLoadPercentage = 0.0;
      for (int i = 0; i < feet.size(); i++)
      {
         RigidBody foot = feet.get(i);
         totalLoadPercentage += footSwitches.get(foot).computeFootLoadPercentage();
      }
      totalLoadPercentageOnFeet.set(totalLoadPercentage);
      boolean areFeetLoadedEnough = totalLoadPercentageOnFeet.getDoubleValue() > loadPercentageOnFeetThresholdForIMUDrift.getDoubleValue();
      isIMUDriftFeetLoadedEnough.set(areFeetLoadedEnough);
      return areFeetLoadedEnough;
   }

   private final FrameVector totalFootAngularVelocityDifferencesFromAverage = new FrameVector();

   private boolean areFeetAngularVelocitiesClose(List<RigidBody> trustedFeet)
   {
      totalFootAngularVelocityDifferencesFromAverage.setToZero();

      for (int i = 0; i < trustedFeet.size(); i++)
      {
         RigidBody foot = trustedFeet.get(i);
         double wx = Math.abs(footAngularVelocityAverage.getX() - footAngularVelocitiesInWorldFilteredX.get(foot).getDoubleValue());
         double wy = Math.abs(footAngularVelocityAverage.getY() - footAngularVelocitiesInWorldFilteredY.get(foot).getDoubleValue());
         double wz = Math.abs(footAngularVelocityAverage.getZ() - footAngularVelocitiesInWorldFilteredZ.get(foot).getDoubleValue());
         totalFootAngularVelocityDifferencesFromAverage.add(wx, wy, wz);
      }

      yoTotalFootAngularVelocityDifferencesFromAverage.set(totalFootAngularVelocityDifferencesFromAverage);
      boolean isAngularVelocityXLowEnough = yoTotalFootAngularVelocityDifferencesFromAverage.getX() < footAngularVelocityDifferenceThresholdToEstimateIMUDrift.getX();
      boolean isAngularVelocityYLowEnough = yoTotalFootAngularVelocityDifferencesFromAverage.getY() < footAngularVelocityDifferenceThresholdToEstimateIMUDrift.getY();
      boolean isAngularVelocityZLowEnough = yoTotalFootAngularVelocityDifferencesFromAverage.getZ() < footAngularVelocityDifferenceThresholdToEstimateIMUDrift.getZ();
      areFootAngularVelocitiesCloseEnough.set(isAngularVelocityXLowEnough && isAngularVelocityYLowEnough && isAngularVelocityZLowEnough);

      return areFootAngularVelocitiesCloseEnough.getBooleanValue();
   }

   /**
    * Estimate the IMU drift yaw using the leg kinematics.
    * @param trustedSide Refers to the foot to trust, set it to null when both feet are trusted.
    */
   private void estimateIMUDriftYaw(List<RigidBody> trustedFeet)
   {
      if (trustedFeet.size() > 1)
      {
         imuDriftYawRate.set(footAngularVelocityAverage.getZ());
      }
      else
      {
         imuDriftYawRate.set(footAngularVelocitiesInWorldFilteredZ.get(trustedFeet.get(0)).getDoubleValue());
      }
      imuDriftYawRateFiltered.update();

      imuDriftYawAngle.add(imuDriftYawRateFiltered.getDoubleValue() * estimatorDT);
      imuDriftYawAngle.set(AngleTools.trimAngleMinusPiToPi(imuDriftYawAngle.getDoubleValue()));

      rootJoint.getRotation(rootJointYawPitchRoll);
      rootJointYawPitchRoll[0] -= imuDriftYawAngle.getDoubleValue();
      rootJointYawPitchRoll[0] = AngleTools.trimAngleMinusPiToPi(rootJointYawPitchRoll[0]);
      rootJointYawAngleCorrected.set(rootJointYawPitchRoll[0]);

      rootJoint.getJointTwist(rootJointTwist);
      rootJointTwist.getAngularPart(rootJointAngularVelocity);
      rootJointAngularVelocity.changeFrame(worldFrame);
      rootJointYawRateCorrected.set(rootJointAngularVelocity.getZ() - imuDriftYawRateFiltered.getDoubleValue());
   }

   private final double[] rootJointYawPitchRoll = new double[] {0.0, 0.0, 0.0};
   private final Twist rootJointTwist = new Twist();
   private final FrameVector rootJointAngularVelocity = new FrameVector();

   private void compensateIMUDriftYaw()
   {
      rootJoint.getRotation(rootJointYawPitchRoll);
      rootJointYawPitchRoll[0] -= imuDriftYawAngle.getDoubleValue();
      rootJointYawPitchRoll[0] = AngleTools.trimAngleMinusPiToPi(rootJointYawPitchRoll[0]);
      rootJointYawAngleCorrected.set(rootJointYawPitchRoll[0]);
      rootJoint.setRotation(rootJointYawPitchRoll[0], rootJointYawPitchRoll[1], rootJointYawPitchRoll[2]);
      rootJoint.getFrameAfterJoint().update();

      rootJoint.getJointTwist(rootJointTwist);
      rootJointTwist.getAngularPart(rootJointAngularVelocity);
      rootJointAngularVelocity.changeFrame(worldFrame);
      rootJointYawRateCorrected.set(rootJointAngularVelocity.getZ() - imuDriftYawRateFiltered.getDoubleValue());
      rootJointAngularVelocity.setZ(rootJointYawRateCorrected.getDoubleValue());
      rootJointAngularVelocity.changeFrame(rootJointFrame);
      rootJointTwist.setAngularPart(rootJointAngularVelocity.getVector());
      rootJoint.setJointTwist(rootJointTwist);
      twistCalculator.compute();
   }

   private void updateFootOrientations()
   {
      for (int i = 0; i < feet.size(); i++)
      {
         RigidBody foot = feet.get(i);
         FrameOrientation footOrientation = footOrientations.get(foot);

         footOrientation.setToZero(footFrames.get(foot));
         footOrientation.changeFrame(worldFrame);

         YoFrameQuaternion footOrientationInWorld = footOrientationsInWorld.get(foot);
         footOrientationInWorld.set(footOrientation);

         FiniteDifferenceAngularVelocityYoFrameVector footAngularVelocityInWorld = footAngularVelocitiesInWorld.get(foot);
         footAngularVelocityInWorld.update();

         footAngularVelocitiesInWorldFilteredX.get(foot).update(footAngularVelocityInWorld.getX());
         footAngularVelocitiesInWorldFilteredY.get(foot).update(footAngularVelocityInWorld.getY());
         footAngularVelocitiesInWorldFilteredZ.get(foot).update(footAngularVelocityInWorld.getZ());
      }

      footAngularVelocityAverage.setToZero();

      for (int i = 0; i < feet.size(); i++)
      {
         RigidBody foot = feet.get(i);

         double footAngularVelocityX = footAngularVelocitiesInWorldFilteredX.get(foot).getDoubleValue();
         double footAngularVelocityY = footAngularVelocitiesInWorldFilteredY.get(foot).getDoubleValue();
         double footAngularVelocityZ = footAngularVelocitiesInWorldFilteredZ.get(foot).getDoubleValue();

         footAngularVelocityAverage.add(footAngularVelocityX, footAngularVelocityY, footAngularVelocityZ);
      }

      footAngularVelocityAverage.scale(1.0 / feet.size());
      yoFootAngularVelocityAverage.set(footAngularVelocityAverage);
   }

   public void resetFootAngularVelocitiesFiltered()
   {
      for (int i = 0; i < feet.size(); i++)
      {
         RigidBody foot = feet.get(i);
         footAngularVelocitiesInWorldFilteredX.get(foot).set(0.0);
         footAngularVelocitiesInWorldFilteredY.get(foot).set(0.0);
         footAngularVelocitiesInWorldFilteredZ.get(foot).set(0.0);
      }
   }
}
