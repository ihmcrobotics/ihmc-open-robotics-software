package us.ihmc.sensorProcessing.outputData;

import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * Output writer running in the estimator thread. 
 * 
 * @author jesper
 *
 */
public interface JointDesiredOutputWriter
{
   /**
    * Set the holder for the desired output values of all joints 
    * 
    * @param jointDesiredOutputList
    */
   void setJointDesiredOutputList(JointDesiredOutputList jointDesiredOutputList);
   
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
    * This function is called in a realtime context.
    * The data from the controller is valid and this function gets called before writeBefore() ever gets called.
    * 
    */
   void initialize();
   
   
   /**
    * Write function that gets called before the estimator has ran.
    * 
    * All joint data is from the previous estimator tick. 
    * initialize() has been called once when this function is called.
    * 
    * @param timestamp Timestamp the estimator tick started
    */
   void writeBefore(long timestamp);
   
   /**
    * Write function that gets called after the estimator has ran. 
    * 
    * All joint data has been updated to the latest measured values.
    * initialize() has been called once when this function is called.
    * 
    */
   void writeAfter();
   
   /**
    * Get the yoVariableRegistry for this output writer
    *  
    * @return The YoVariableRegistry for this output writer.
    */
   YoVariableRegistry getYoVariableRegistry();
}
