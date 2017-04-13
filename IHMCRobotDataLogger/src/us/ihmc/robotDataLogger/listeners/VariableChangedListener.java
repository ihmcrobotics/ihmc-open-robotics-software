package us.ihmc.robotDataLogger.listeners;

/**
 * Listener interface for changed variables
 * 
 * @author jesper
 *
 */
public interface VariableChangedListener
{
   /**
    * A variable got changed by a remote consumer 
    * 
    * @param id ID of the YoVariable
    * @param newValue Desired value
    */
   public void changeVariable(int id, double newValue);
}
