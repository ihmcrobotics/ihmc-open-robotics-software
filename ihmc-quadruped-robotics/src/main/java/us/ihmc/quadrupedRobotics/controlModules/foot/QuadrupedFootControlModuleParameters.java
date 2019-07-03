package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.ParameterizedPID3DGains;
import us.ihmc.robotics.dataStructures.parameters.FrameParameterVector3D;
import us.ihmc.robotics.dataStructures.parameters.ParameterVector3D;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedFootControlModuleParameters
{
   // final registry
   private final YoVariableRegistry finalRegistry = new YoVariableRegistry("QuadrupedFootControlModule");

   private static final int defaultTouchdownTriggerWindow = 1;

   // swing state parameters
   private final ParameterizedPID3DGains solePositionGains;
   private final Vector3DReadOnly solePositionWeights = new ParameterVector3D("solePositionWeights", new Vector3D(30.0, 30.0, 30.0), finalRegistry);
   private final DoubleParameter minimumStepAdjustmentFractionRemaining = new DoubleParameter("minimumStepAdjustmentFractionRemaining", finalRegistry, 0.05);
   private final DoubleParameter fractionThroughSwingForAdjustment = new DoubleParameter("fractionThroughSwingForAdjustment", finalRegistry, 0.2);
   private final DoubleParameter stepGoalOffsetZParameter = new DoubleParameter("stepGoalOffsetZ", finalRegistry, 0.0);


   private final Vector3D defaultTouchdownVelocity = new Vector3D(0.0, 0.0, 0.0);
   private final FrameVector3DReadOnly touchdownVelocity = new FrameParameterVector3D("swingTouchdownVelocity", ReferenceFrame.getWorldFrame(),
                                                                                      defaultTouchdownVelocity, finalRegistry);
   private final FrameVector3DReadOnly touchdownAcceleration = new FrameParameterVector3D("swingTouchdownAcceleration", ReferenceFrame.getWorldFrame(),
                                                                                          defaultTouchdownVelocity, finalRegistry);
   private final DoubleProvider percentPastSwingForDone = new DoubleParameter("percentPastSwingForDone", finalRegistry, 0.0);
   private final DoubleProvider minHeightDifferenceForObstacleClearance = new DoubleParameter("minHeightDifferenceForObstacleClearance", finalRegistry, 0.04);
   private final DoubleProvider minPhaseThroughSwingForContact = new DoubleParameter("minPhaseThroughSwingForContact", finalRegistry, 0.8);
   private final DoubleProvider fractionOfSwingForBlending = new DoubleParameter("fractionOfSwingForBlending", finalRegistry, 0.8);

   private final BooleanProvider isSwingSpeedUpEnabled = new BooleanParameter("isSwingSpeedUpEnabled", finalRegistry, false);
   private final DoubleProvider minSwingTimeForDisturbanceRecovery = new DoubleParameter("minSwingTimeForDisturbanceRecovery", finalRegistry, 0.2);
   private final DoubleProvider minRequiredSpeedUpFactor = new DoubleParameter("minRequiredSpeedUpFactor", finalRegistry, 1.05);

   private final DoubleProvider flatWaypointProportion = new DoubleParameter("swingFlatWaypointProportion", finalRegistry, 0.5);
   private final DoubleProvider waypointProportion0 = new DoubleParameter("swingWaypointProportion0", finalRegistry, 0.33);
   private final DoubleProvider waypointProportion1 = new DoubleParameter("swingWaypointProportion1", finalRegistry, 0.66);
   private final DoubleProvider obstacleClearanceWaypointProportion0 = new DoubleParameter("swingObstacleClearanceWaypointProportion0", finalRegistry, 0.25);
   private final DoubleProvider obstacleClearanceWaypointProportion1 = new DoubleParameter("swingObstacleClearanceWaypointProportion1", finalRegistry, 0.75);

   // support state parameters
   private final ParameterizedPID3DGains holdPositionGains;
   private final Vector3DReadOnly supportFootWeights = new ParameterVector3D("supportFootWeights", new Vector3D(10.0, 10.0, 10.0), finalRegistry);
   private final DoubleParameter barelyLoadedWindowLength = new DoubleParameter("footBarelyLoadedWindowLength", finalRegistry, 0.05);
   private final DoubleParameter footBarelyLoadedThreshold = new DoubleParameter("footFootBarelyLoadedThreshold", finalRegistry, 0.10);
   private final DoubleParameter footFullyLoadedThreshold = new DoubleParameter("footFullyLoadedThreshold", finalRegistry, 0.15);

   private final DoubleProvider minimumTimeInSupportState = new DoubleParameter("minimumTimeInSupportState", finalRegistry, 0.05);
   private final DoubleProvider maximumPhaseThroughStepToAllowStart = new DoubleParameter("maximumPhaseThroughStepToAllowStart", finalRegistry, 0.5);

   private final DoubleProvider footVelocityThresholdForSlipping = new DoubleParameter("footVelocityThresholdForSlipping", finalRegistry, 0.25);
   private final DoubleProvider footVelocityThresholdForNotSlipping = new DoubleParameter("footVelocityThresholdForNotSlipping", finalRegistry, 0.1);

   private final DoubleProvider coefficientOfFrictionWhenSlipping = new DoubleParameter("coefficientOfFrictionWhenSlipping", finalRegistry, 0.6);
   private final DoubleProvider coefficientOfFrictionWhenNotSlipping = new DoubleParameter("coefficientOfFrictionWhenNotSlipping", finalRegistry, 0.9);



   public QuadrupedFootControlModuleParameters()
   {
      DefaultPID3DGains solePositionDefaultGains = new DefaultPID3DGains();
      solePositionDefaultGains.setProportionalGains(10000.0, 10000.0, 5000.0);
      solePositionDefaultGains.setDerivativeGains(200.0, 200.0, 200.0);
      solePositionDefaultGains.setIntegralGains(0.0, 0.0, 0.0, 0.0);
      solePositionGains = new ParameterizedPID3DGains("_solePosition", GainCoupling.NONE, false, solePositionDefaultGains, finalRegistry);

      DefaultPID3DGains holdPositionDefaultGains = new DefaultPID3DGains();
      holdPositionDefaultGains.setProportionalGains(10000.0, 10000.0, 5000.0);
      holdPositionDefaultGains.setDerivativeGains(200.0, 200.0, 200.0);
      holdPositionDefaultGains.setIntegralGains(0.0, 0.0, 0.0, 0.0);
      holdPositionGains = new ParameterizedPID3DGains("_holdPosition", GainCoupling.NONE, false, holdPositionDefaultGains, finalRegistry);
   }
   
   
   public YoVariableRegistry getYoVariableRegistry()
   {
      return finalRegistry;
   }

   public PID3DGainsReadOnly getSolePositionGains()
   {
      return solePositionGains;
   }

   public Vector3DReadOnly getSolePositionWeights()
   {
      return solePositionWeights;
   }

   public double getMinimumStepAdjustmentFractionRemaining()
   {
      return minimumStepAdjustmentFractionRemaining.getValue();
   }

   public double getFractionThroughSwingForAdjustment()
   {
      return fractionThroughSwingForAdjustment.getValue();
   }

   public double getFractionOfSwingForBlending()
   {
      return fractionOfSwingForBlending.getValue();
   }

   public double getStepGoalOffsetZParameter()
   {
      return stepGoalOffsetZParameter.getValue();
   }

   public BooleanProvider getIsSwingSpeedUpEnabled()
   {
      return isSwingSpeedUpEnabled;
   }

   public double getPercentPastSwingForDone()
   {
      return percentPastSwingForDone.getValue();
   }

   public double getMinHeightDifferenceForObstacleClearance()
   {
      return minHeightDifferenceForObstacleClearance.getValue();
   }

   public double getMinPhaseThroughSwingForContact()
   {
      return minPhaseThroughSwingForContact.getValue();
   }

   public double getMinSwingTimeForDisturbanceRecovery()
   {
      return minSwingTimeForDisturbanceRecovery.getValue();
   }

   public double getMinRequiredSpeedUpFactor()
   {
      return minRequiredSpeedUpFactor.getValue();
   }

   public double getFlatSwingWaypointProportion()
   {
      return flatWaypointProportion.getValue();
   }

   public double getSwingWaypointProportion0()
   {
      return waypointProportion0.getValue();
   }

   public double getSwingWaypointProportion1()
   {
      return waypointProportion1.getValue();
   }

   public double getSwingObstacleClearanceWaypointProportion0()
   {
      return obstacleClearanceWaypointProportion0.getValue();
   }

   public double getSwingObstacleClearanceWaypointProportion1()
   {
      return obstacleClearanceWaypointProportion1.getValue();
   }

   public Vector3DReadOnly getSupportFootWeights()
   {
      return supportFootWeights;
   }

   public PID3DGainsReadOnly getHoldPositionGains()
   {
      return holdPositionGains;
   }

   public DoubleProvider getBarelyLoadedWindowLength()
   {
      return barelyLoadedWindowLength;
   }

   public DoubleProvider getBarelyLoadedThreshold()
   {
      return footBarelyLoadedThreshold;
   }

   public DoubleProvider getFullyLoadedThreshold()
   {
      return footFullyLoadedThreshold;
   }

   public double getFootVelocityThresholdForSlipping()
   {
      return footVelocityThresholdForSlipping.getValue();
   }

   public double getFootVelocityThresholdForNotSlipping()
   {
      return footVelocityThresholdForNotSlipping.getValue();
   }

   public double getCoefficientOfFrictionWhenSlipping()
   {
      return coefficientOfFrictionWhenSlipping.getValue();
   }

   public double getCoefficientOfFrictionWhenNotSlipping()
   {
      return coefficientOfFrictionWhenNotSlipping.getValue();
   }

   public double getMinimumTimeInSupportState()
   {
      return minimumTimeInSupportState.getValue();
   }

   public double getMaximumPhaseThroughStepToAllowStart()
   {
      return maximumPhaseThroughStepToAllowStart.getValue();
   }

   public FrameVector3DReadOnly getTouchdownVelocity()
   {
      return touchdownVelocity;
   }

   public FrameVector3DReadOnly getTouchdownAcceleration()
   {
      return touchdownAcceleration;
   }

   public static int getDefaultTouchdownTriggerWindow()
   {
      return defaultTouchdownTriggerWindow;
   }
}
