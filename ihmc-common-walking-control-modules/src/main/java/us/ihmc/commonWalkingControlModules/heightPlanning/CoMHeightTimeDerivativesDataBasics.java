package us.ihmc.commonWalkingControlModules.heightPlanning;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public interface CoMHeightTimeDerivativesDataBasics extends CoMHeightTimeDerivativesDataReadOnly
{
   void setComHeight(ReferenceFrame referenceFrame, double comHeight);

   void setComHeightVelocity(double comHeightVelocity);

   void setComHeightAcceleration(double comHeightAcceleration);

   void setComHeightJerk(double comHeightJerk);

   default void set(CoMHeightTimeDerivativesDataReadOnly heightZData)
   {
      setComHeight(heightZData.getReferenceFrame(), heightZData.getComHeightInFrame());
      setComHeightVelocity(heightZData.getComHeightVelocity());
      setComHeightAcceleration(heightZData.getComHeightAcceleration());
      setComHeightJerk(heightZData.getComHeightJerk());
   }

}
