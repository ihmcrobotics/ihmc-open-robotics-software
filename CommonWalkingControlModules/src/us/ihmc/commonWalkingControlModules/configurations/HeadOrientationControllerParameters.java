package us.ihmc.commonWalkingControlModules.configurations;

public interface HeadOrientationControllerParameters
{
   public abstract String[] getHeadOrientationControlJointNames();
   
   public abstract String getJointNameForExtendedPitchRange();
   
   public abstract double getUpperNeckPitchLimit();
   
   public abstract double getLowerNeckPitchLimit();
   
   public abstract double getHeadYawLimit();

   public abstract double getHeadRollLimit();
}
