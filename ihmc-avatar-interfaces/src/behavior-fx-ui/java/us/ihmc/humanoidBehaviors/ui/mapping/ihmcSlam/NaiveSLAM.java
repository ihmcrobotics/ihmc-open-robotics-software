package us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

public class NaiveSLAM extends IhmcSLAM
{
   public NaiveSLAM(double octreeResolution)
   {
      super(octreeResolution);
   }

   @Override
   protected RigidBodyTransformReadOnly computeFrameCorrectionTransformer(IhmcSLAMFrame frame)
   {
      return new RigidBodyTransform();
   }
}
