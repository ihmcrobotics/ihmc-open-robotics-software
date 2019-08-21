package us.ihmc.sensorProcessing.parameters;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public interface IMUSensorInformation
{
   public String[] getIMUSensorsToUseInStateEstimator();

   public String getPrimaryBodyImu();
}
