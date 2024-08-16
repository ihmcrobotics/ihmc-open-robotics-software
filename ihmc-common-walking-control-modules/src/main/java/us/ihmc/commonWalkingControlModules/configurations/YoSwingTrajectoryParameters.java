package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.robotics.dataStructures.parameters.FrameParameterVector3D;
import us.ihmc.robotics.dataStructures.parameters.ParameterVector3D;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

public class YoSwingTrajectoryParameters
{
   private final DoubleParameter defaultSwingStepUpHeight;
   private final DoubleParameter defaultSwingStepDownHeight;
   private final BooleanParameter addOrientationMidpointForClearance;
   private final BooleanParameter addFootPitchToAvoidHeelStrikeWhenSteppingForwardAndDown;
   private final DoubleParameter fractionOfSwingToPitchFootDown;
   private final DoubleParameter footPitchAngleToAvoidHeelStrike;
   private final DoubleParameter midpointOrientationInterpolationForClearance;

   private final DoubleParameter minHeightDifferenceForObstacleClearance;
   private final DoubleParameter finalSwingHeightOffset;

   private final List<DoubleProvider> defaultWaypointProportions = new ArrayList<>();
   private final List<DoubleProvider> defaultStepUpWaypointProportions = new ArrayList<>();
   private final List<DoubleProvider> defaultStepDownWaypointProportions = new ArrayList<>();

   private final DoubleProvider minLiftOffVerticalVelocity;
   private final ParameterVector3D touchdownVelocityWeight;
   private final FrameParameterVector3D touchdownVelocity;
   private final FrameParameterVector3D touchdownAcceleration;
   private final DoubleProvider finalCoMVelocityInjectionRatio;
   private final DoubleProvider finalCoMAccelerationInjectionRatio;

   private final BooleanProvider ignoreInitialAngularVelocityZ;
   private final DoubleProvider maxInitialLinearVelocityMagnitude;
   private final DoubleProvider maxInitialAngularVelocityMagnitude;

   private final BooleanProvider addLiftOffKneeAcceleration;
   private final DoubleProvider liftOffKneeDesiredVelocity;
   private final DoubleProvider liftOffPhaseDuration;
   private final DoubleProvider liftOffKd;

   private final DoubleProvider pelvisVelocityInjectionRatio;

   public YoSwingTrajectoryParameters(String namePrefix, WalkingControllerParameters walkingControllerParameters, YoRegistry registry)
   {
      this(namePrefix, walkingControllerParameters, walkingControllerParameters.getSwingTrajectoryParameters(), registry);
   }

   public YoSwingTrajectoryParameters(String namePrefix,
                                      WalkingControllerParameters walkingControllerParameters,
                                      SwingTrajectoryParameters parameters,
                                      YoRegistry registry)
   {
      defaultSwingStepUpHeight = new DoubleParameter(namePrefix + "DefaultSwingStepUpHeight", registry, parameters.getDefaultSwingStepUpHeight());
      defaultSwingStepDownHeight = new DoubleParameter(namePrefix + "DefaultSwingStepDownHeight", registry, parameters.getDefaultSwingStepDownHeight());
      addOrientationMidpointForClearance = new BooleanParameter(namePrefix + "AddOrientationMidpointForClearance",
                                                                registry,
                                                                parameters.addOrientationMidpointForObstacleClearance());
      midpointOrientationInterpolationForClearance = new DoubleParameter(namePrefix + "MidpointOrientationInterpolationForClearance",
                                                                         registry,
                                                                         parameters.midpointOrientationInterpolationForObstacleClearance());
      addFootPitchToAvoidHeelStrikeWhenSteppingForwardAndDown = new BooleanParameter(namePrefix + "AddFootPitchToAvoidHeelStrikeWhenSteppingDown",
                                                                                     registry,
                                                                                     parameters.addFootPitchToAvoidHeelStrikeWhenSteppingForwardAndDown());
      fractionOfSwingToPitchFootDown = new DoubleParameter(namePrefix + "FractionOfSwingToPitchFootDown",
                                                           registry,
                                                           parameters.getFractionOfSwingToPitchFootDown());
      footPitchAngleToAvoidHeelStrike = new DoubleParameter(namePrefix + "FootPitchAngleToAvoidHeelStrike",
                                                            registry,
                                                            parameters.getFootPitchAngleToAvoidHeelStrike());

      minHeightDifferenceForObstacleClearance = new DoubleParameter(namePrefix + "MinHeightDifferenceForObstacleClearance",
                                                                    registry,
                                                                    parameters.getMinHeightDifferenceForStepUpOrDown());

      int numberWaypoints = 2;
      double[] defaultWaypointProportions = parameters.getSwingWaypointProportions();
      double[] defaultStepUpWaypointProportions = parameters.getSwingStepUpWaypointProportions();
      double[] defaultStepDownWaypointProportions = parameters.getSwingStepDownWaypointProportions();

      for (int i = 0; i < numberWaypoints; i++)
      {
         DoubleParameter waypointProportion = new DoubleParameter(namePrefix + "WaypointProportion" + i, registry, defaultWaypointProportions[i]);
         DoubleParameter stepUpWaypointProportion = new DoubleParameter(namePrefix + "StepUpWaypointProportion" + i,
                                                                        registry,
                                                                        defaultStepUpWaypointProportions[i]);
         DoubleParameter stepDownWaypointProportion = new DoubleParameter(namePrefix + "StepDownWaypointProportion" + i,
                                                                          registry,
                                                                          defaultStepDownWaypointProportions[i]);
         this.defaultWaypointProportions.add(waypointProportion);
         this.defaultStepUpWaypointProportions.add(stepUpWaypointProportion);
         this.defaultStepDownWaypointProportions.add(stepDownWaypointProportion);
      }

      this.touchdownVelocityWeight = new ParameterVector3D(namePrefix + "TouchdownVelocityWeight", parameters.getTouchdownVelocityWeight(), registry);

      finalSwingHeightOffset = new DoubleParameter(namePrefix + "FinalHeightOffset", registry, parameters.getDesiredTouchdownHeightOffset());

      Vector3D defaultTouchdownVelocity = new Vector3D(0.0, 0.0, parameters.getDesiredTouchdownVelocity());
      touchdownVelocity = new FrameParameterVector3D(namePrefix + "TouchdownVelocity", ReferenceFrame.getWorldFrame(), defaultTouchdownVelocity, registry);
      minLiftOffVerticalVelocity = new DoubleParameter(namePrefix + "MinLiftOffVerticalVelocity", registry, parameters.getMinLiftOffVerticalVelocity());
      finalCoMVelocityInjectionRatio = new DoubleParameter(namePrefix + "FinalCoMVelocityInjectionRatio",
                                                           registry,
                                                           parameters.getFinalCoMVelocityInjectionRatio());
      finalCoMAccelerationInjectionRatio = new DoubleParameter(namePrefix + "finalCoMAccelerationInjectionRatio",
                                                               registry,
                                                               parameters.getFinalCoMAccelerationInjectionRatio());

      Vector3D defaultTouchdownAcceleration = new Vector3D(0.0, 0.0, parameters.getDesiredTouchdownAcceleration());
      touchdownAcceleration = new FrameParameterVector3D(namePrefix + "TouchdownAcceleration",
                                                         ReferenceFrame.getWorldFrame(),
                                                         defaultTouchdownAcceleration,
                                                         registry);

      ignoreInitialAngularVelocityZ = new BooleanParameter(namePrefix + "IgnoreInitialAngularVelocityZ",
                                                           registry,
                                                           walkingControllerParameters.ignoreSwingInitialAngularVelocityZ());
      maxInitialLinearVelocityMagnitude = new DoubleParameter(namePrefix + "MaxInitialLinearVelocityMagnitude",
                                                              registry,
                                                              walkingControllerParameters.getMaxSwingInitialLinearVelocityMagnitude());
      maxInitialAngularVelocityMagnitude = new DoubleParameter(namePrefix + "MaxInitialAngularVelocityMagnitude",
                                                               registry,
                                                               walkingControllerParameters.getMaxSwingInitialAngularVelocityMagnitude());

      addLiftOffKneeAcceleration = new BooleanParameter(namePrefix + "AddLiftOffKneeAcceleration", registry, false);
      liftOffKneeDesiredVelocity = new DoubleParameter(namePrefix + "LiftOffKneeDesiredVelocity", registry, 3.0);
      liftOffPhaseDuration = new DoubleParameter(namePrefix + "LiftOffPhaseDuration", registry, 0.05);
      liftOffKd = new DoubleParameter(namePrefix + "LiftOffKneeKd", registry, 50.0);

      pelvisVelocityInjectionRatio = new DoubleParameter(namePrefix + "PelvisVelocityInjectionRatio", registry, parameters.getPelvisVelocityInjectionRatio());
   }

   public double getDefaultSwingStepUpHeight()
   {
      return defaultSwingStepUpHeight.getValue();
   }

   public double getDefaultSwingStepDownHeight()
   {
      return defaultSwingStepDownHeight.getValue();
   }

   public boolean addOrientationMidpointForObstacleClearance()
   {
      return addOrientationMidpointForClearance.getValue();
   }

   public double getMidpointOrientationInterpolationForObstacleClearance()
   {
      return midpointOrientationInterpolationForClearance.getValue();
   }

   public boolean addFootPitchToAvoidHeelStrikeWhenSteppingForwardAndDown()
   {
      return addFootPitchToAvoidHeelStrikeWhenSteppingForwardAndDown.getValue();
   }

   public double getFractionOfSwingToPitchFootDown()
   {
      return fractionOfSwingToPitchFootDown.getValue();
   }

   public double getFootPitchAngleToAvoidHeelStrike()
   {
      return footPitchAngleToAvoidHeelStrike.getValue();
   }

   public double getMinHeightDifferenceForStepUpOrDown()
   {
      return minHeightDifferenceForObstacleClearance.getValue();
   }

   public List<DoubleProvider> getSwingWaypointProportions()
   {
      return defaultWaypointProportions;
   }

   public List<DoubleProvider> getSwingStepUpWaypointProportions()
   {
      return defaultStepUpWaypointProportions;
   }

   public List<DoubleProvider> getSwingStepDownWaypointProportions()
   {
      return defaultStepDownWaypointProportions;
   }

   public Tuple3DReadOnly getTouchdownVelocityWeight()
   {
      return touchdownVelocityWeight;
   }

   public double getDesiredTouchdownHeightOffset()
   {
      return finalSwingHeightOffset.getValue();
   }

   public FrameVector3DReadOnly getDesiredTouchdownVelocity()
   {
      return touchdownVelocity;
   }

   public FrameVector3DReadOnly getDesiredTouchdownAcceleration()
   {
      return touchdownAcceleration;
   }

   public double getFinalCoMVelocityInjectionRatio()
   {
      return finalCoMVelocityInjectionRatio.getValue();
   }

   public double getFinalCoMAccelerationInjectionRatio()
   {
      return finalCoMAccelerationInjectionRatio.getValue();
   }

   public boolean ignoreSwingInitialAngularVelocityZ()
   {
      return ignoreInitialAngularVelocityZ.getValue();
   }

   public double getPelvisVelocityInjectionRatio()
   {
      return pelvisVelocityInjectionRatio.getValue();
   }

   public double getMaxSwingInitialLinearVelocityMagnitude()
   {
      return maxInitialLinearVelocityMagnitude.getValue();
   }

   public double getMaxSwingInitialAngularVelocityMagnitude()
   {
      return maxInitialAngularVelocityMagnitude.getValue();
   }

   public double getMinLiftOffVerticalVelocity()
   {
      return minLiftOffVerticalVelocity.getValue();
   }

   public boolean addLiftOffKneeAcceleration()
   {
      return addLiftOffKneeAcceleration.getValue();
   }

   public double getLiftOffKneeDesiredVelocity()
   {
      return liftOffKneeDesiredVelocity.getValue();
   }

   public double getLiftOffPhaseDuration()
   {
      return liftOffPhaseDuration.getValue();
   }

   public double getLiftOffKd()
   {
      return liftOffKd.getValue();
   }
}
