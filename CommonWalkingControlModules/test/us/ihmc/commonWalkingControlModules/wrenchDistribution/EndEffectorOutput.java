package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialForceVector;

public class EndEffectorOutput
{
   private final SpatialForceVector resultingExternalForceVector;
   public EndEffectorOutput(ReferenceFrame comFrame)
   {
      resultingExternalForceVector = new SpatialForceVector(comFrame);
   }
   public void setExternallyActingSpatialForceVector(SpatialForceVector spatialForceVector)
   {
      resultingExternalForceVector.set(spatialForceVector);
   }
   public void packExternallyActingSpatialForceVector(SpatialForceVector vectorToPack)
   {
      vectorToPack.set(resultingExternalForceVector);
   }
}
