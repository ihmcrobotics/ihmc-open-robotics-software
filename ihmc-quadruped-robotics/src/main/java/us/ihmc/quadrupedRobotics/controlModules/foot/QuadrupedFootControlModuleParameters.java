package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.ParameterizedPID3DGains;
import us.ihmc.robotics.dataStructures.parameters.ParameterVector3D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedFootControlModuleParameters
{
   // final registry
   private final YoVariableRegistry finalRegistry = new YoVariableRegistry("QuadrupedFootControlModule");

   private static final int defaultTouchdownTriggerWindow = 1;

   // final parameters
   private final ParameterizedPID3DGains solePositionGains;
   private final ParameterizedPID3DGains holdPositionGains;
   private final Vector3DReadOnly solePositionWeights = new ParameterVector3D("solePositionWeights", new Vector3D(30.0, 30.0, 30.0), finalRegistry);
   private final Vector3DReadOnly supportFootWeights = new ParameterVector3D("supportFootWeights", new Vector3D(10.0, 10.0, 10.0), finalRegistry);
   private final DoubleParameter minimumStepAdjustmentTimeRemaining = new DoubleParameter("minimumStepAdjustmentFractionRemaining", finalRegistry, 0.05);
   private final DoubleParameter fractionThroughSwingForAdjustment = new DoubleParameter("fractionThroughSwingForAdjustment", finalRegistry, 0.2);
   private final DoubleParameter stepGoalOffsetZParameter = new DoubleParameter("stepGoalOffsetZ", finalRegistry, 0.0);

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

   public PID3DGainsReadOnly getHoldPositionGains()
   {
      return holdPositionGains;
   }

   public Vector3DReadOnly getSolePositionWeights()
   {
      return solePositionWeights;
   }

   public Vector3DReadOnly getSupportFootWeights()
   {
      return supportFootWeights;
   }

   public double getMinimumStepAdjustmentFractionRemaining()
   {
      return minimumStepAdjustmentTimeRemaining.getValue();
   }

   public double getFractionThroughSwingForAdjustment()
   {
      return fractionThroughSwingForAdjustment.getValue();
   }

   public double getStepGoalOffsetZParameter()
   {
      return stepGoalOffsetZParameter.getValue();
   }


   public static int getDefaultTouchdownTriggerWindow()
   {
      return defaultTouchdownTriggerWindow;
   }
}
