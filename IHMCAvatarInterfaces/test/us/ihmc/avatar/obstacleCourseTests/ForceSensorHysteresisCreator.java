package us.ihmc.avatar.obstacleCourseTests;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.GlitchFilteredBooleanYoVariable;
import us.ihmc.robotics.robotController.ModularRobotController;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;

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
   private DenseMatrix64F wrench = new DenseMatrix64F(Wrench.SIZE,1);
   private double tmpHysteresis = 0;
   private DoubleYoVariable hysteresisInZDirection;
   
   private boolean hasNormalForceGonePastLimit = false;
   private final GlitchFilteredBooleanYoVariable isForcePastThresholdFiltered;
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
      
      hysteresisInZDirection = new DoubleYoVariable(parentJointName + "ForceSensorZHysteresis", registry);
      hysteresisInZDirection.set(0);
      
      this.isForcePastThresholdFiltered = new GlitchFilteredBooleanYoVariable(parentJointName + "ForceSensorZHysteresisIsForcePastThreshold", registry, ITERS_BEFORE_HYSTERESIS_TRIGGERS);
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
