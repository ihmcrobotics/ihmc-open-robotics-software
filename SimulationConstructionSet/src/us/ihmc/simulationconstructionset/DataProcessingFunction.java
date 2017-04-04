package us.ihmc.simulationconstructionset;


/**
 *
 * <p>DataProcessingFunction</p>
 *
 * <p>A function to be applied to the recorded simulation data from the in point to the out point.</p>
 *
 */
public interface DataProcessingFunction
{
   public void initializeProcessing();
   /**
    * Defines the function to be applied to the data.
    */
   public void processData();
}
