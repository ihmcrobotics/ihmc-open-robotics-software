package us.ihmc.exampleSimulations.fourBarLinkage;

import us.ihmc.graphics3DAdapter.utils.Pair;

public class FourBarLinkageParameters
{
   public double linkageLength_1, linkageLength_2, linkageLength_3, linkageLength_4;
   public double damping_0, damping_1, damping_2, damping_3;
   public double mass_1, mass_2, mass_3, mass_4;
   public double radius_1, radius_2, radius_3, radius_4;
   public double initial_angle_0, initial_angle_1, initial_angle_2;

   /**
    * If the four bar linkage is actuated, it's position is specified here,
    * with the Integer representing the link number and Double being the offset
    * from the link's parent joint
    */
   public Pair<Integer, Double> actuatorPosition_1, actuatorPosition_2;
}
