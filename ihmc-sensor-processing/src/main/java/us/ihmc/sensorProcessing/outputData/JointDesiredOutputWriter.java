package us.ihmc.sensorProcessing.outputData;

import us.ihmc.commons.robotics.outputData.JointDesiredOutputListBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

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
   void setJointDesiredOutputList(JointDesiredOutputListBasics jointDesiredOutputList);

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
    * @return The YoRegistry for this output writer.
    */
   YoRegistry getYoVariableRegistry();
}
