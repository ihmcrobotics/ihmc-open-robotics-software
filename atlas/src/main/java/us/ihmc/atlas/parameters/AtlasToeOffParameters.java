package us.ihmc.atlas.parameters;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;

public class AtlasToeOffParameters extends ToeOffParameters
{
   private final AtlasJointMap jointMap;

   public AtlasToeOffParameters(AtlasJointMap jointMap)
   {
      this.jointMap = jointMap;
   }

   /** {@inheritDoc} */
   @Override
   public boolean doToeOffIfPossible()
   {
      return true;
   }

   @Override
   public boolean doToeOffIfPossibleInSingleSupport()
   {
      return false;
   }

   @Override
   public boolean doToeOffWhenHittingTrailingKneeLowerLimit()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinStepLengthForToeOff()
   {
      return jointMap.getPhysicalProperties().getFootLengthForControl();
   }

   /** {@inheritDoc} */
   @Override
   public boolean doToeOffWhenHittingAnkleLimit()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   public double getAnkleLowerLimitToTriggerToeOff()
   {
      return -1.0;
   }

   @Override
   public double getKneeUpperLimitToTriggerToeOff()
   {
      return 2.0;
   }
}
