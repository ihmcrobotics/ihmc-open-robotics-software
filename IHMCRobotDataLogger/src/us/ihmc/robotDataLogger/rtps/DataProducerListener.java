package us.ihmc.robotDataLogger.rtps;

/**
 * Listener interface for changed variables
 * 
 * @author jesper
 *
 */
public interface DataProducerListener
{
   /**
    * A variable got changed by a remote consumer 
    * 
    * @param id ID of the YoVariable
    * @param newValue Desired value
    */
   public void changeVariable(int id, double newValue);
}
