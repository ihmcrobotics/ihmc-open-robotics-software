package us.ihmc.avatar.obstacleCourseTests;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.yoVariables.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.robotController.ModularRobotController;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * A fairly crude implementation of injecting hysteresis into a force sensor signal.
 */

public class ForceSensorHysteresisCreator extends ModularRobotController
{
   private static final double HYSTERESIS_PERCENT_OF_LOAD = 0.1;
   private static final double PERCENT_OF_FULL_WEIGHT_TO_TRIGGER_HYSTERESIS = 85;
   private static final int ITERS_BEFORE_HYSTERESIS_TRIGGERS = 150;
   
   private final WrenchCalculatorInterface wrenchCalculatorInterface;
   private final double totalRobotWeightInNewtons;
   private DMatrixRMaj wrench = new DMatrixRMaj(Wrench.SIZE,1);
   private double tmpHysteresis = 0;
   private YoDouble hysteresisInZDirection;
   
   private boolean hasNormalForceGonePastLimit = false;
   private final GlitchFilteredYoBoolean isForcePastThresholdFiltered;
   private boolean unfilteredIsForcePastThresh = false;
   private int hysteresisSampleCounter = 0;
   private double normalForceThreshold;
   
   public ForceSensorHysteresisCreator(double totalRobotMass, String parentJointName, WrenchCalculatorInterface wrenchCalculatorInterface)
   {
      super(wrenchCalculatorInterface.getName() + "HysteresisCreator");
      
      this.wrenchCalculatorInterface = wrenchCalculatorInterface;
      this.wrenchCalculatorInterface.setDoWrenchCorruption(true);
      this.totalRobotWeightInNewtons = totalRobotMass * 9.81;
      this.normalForceThreshold = totalRobotWeightInNewtons * PERCENT_OF_FULL_WEIGHT_TO_TRIGGER_HYSTERESIS/100;
      
      hysteresisInZDirection = new YoDouble(parentJointName + "ForceSensorZHysteresis", registry);
      hysteresisInZDirection.set(0);
      
      this.isForcePastThresholdFiltered = new GlitchFilteredYoBoolean(parentJointName + "ForceSensorZHysteresisIsForcePastThreshold", registry, ITERS_BEFORE_HYSTERESIS_TRIGGERS);
   }

   @Override
   public void doControl()
   {
      super.doControl();
      
      wrench = wrenchCalculatorInterface.getWrench();
      
      isNormalForcePastHysteresisThreshold();
      
      if(isForcePastThresholdFiltered.getBooleanValue() && unfilteredIsForcePastThresh) //take lots of true readings to turn on, only 1 to turn off
      {  
         hysteresisSampleCounter++;
         tmpHysteresis += wrench.get(Wrench.SIZE-1);
         hasNormalForceGonePastLimit = true;
      }
      else
      {
         if(hasNormalForceGonePastLimit)
         {
            hysteresisInZDirection.set(hysteresisInZDirection.getDoubleValue() + (tmpHysteresis/hysteresisSampleCounter)*HYSTERESIS_PERCENT_OF_LOAD/100);
            wrenchCalculatorInterface.corruptWrenchElement(Wrench.SIZE-1,hysteresisInZDirection.getDoubleValue());
            
            tmpHysteresis = 0;
            hysteresisSampleCounter = 0;
            hasNormalForceGonePastLimit = false;
         }
      } 
   }
   
   private void isNormalForcePastHysteresisThreshold()
   {
      isForcePastThresholdFiltered.update((Math.abs(wrench.get(Wrench.SIZE-1)) - Math.abs(hysteresisInZDirection.getDoubleValue())) > normalForceThreshold);
      unfilteredIsForcePastThresh = (Math.abs(wrench.get(Wrench.SIZE-1)) - Math.abs(hysteresisInZDirection.getDoubleValue())) > normalForceThreshold;
   }
}
