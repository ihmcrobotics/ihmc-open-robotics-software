package us.ihmc.sensorProcessing.outputData;

import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * Output writer running in the estimator thread. 
 * 
 * @author jesper
 *
 */
public interface LowLevelOutputWriter
{
   /**
    * Set the holder for the desired output values of all joints 
    * 
    * @param lowLevelDataHolder
    */
   void setLowLevelOneDoFJointDesiredDataHolderList(LowLevelOneDoFJointDesiredDataHolderList lowLevelDataHolder);
   
   /**
    * Set the holder for the force sensor data holder.
    * 
    * The data in this holder come directly from the estimator.
    * 
    * @param forceSensorDataHolderForController
    */
   void setForceSensorDataHolder(ForceSensorDataHolderReadOnly forceSensorDataHolderForEstimator);

   /**
    * Called the first time the outputwriter runs.
    * 
    * This function is called in a realtime context. The data from the controller is valid. 
    * 
    */
   void initialize();
   
   /**
    * Write controller data to the output structures.
    */
   void write();
   
   /**
    * Get the yoVariableRegistry for this output writer
    *  
    * @return The YoVariableRegistry for this output writer.
    */
   YoVariableRegistry getYoVariableRegistry();
}
