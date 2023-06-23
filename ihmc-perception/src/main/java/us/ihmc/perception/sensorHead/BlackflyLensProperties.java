package us.ihmc.perception.sensorHead;

import us.ihmc.perception.spinnaker.BlackflyModelProperties;

public enum BlackflyLensProperties
{
   BFLY_U3_23S6C_FE185C086HA_1(451.13411,
                               451.23058,
                               934.69385,
                               578.91155,
                               0.0066063,
                               0.0103141,
                               -0.0056699,
                               0.0007021),
   /**
    * These Blackfly S 27S5C parameters were calculated and tuned by @dcalvert, @danderson, and @tbialek on 6/23/2023
    * with a setup with 4 ArUco markers out in the main lab space, one occupying each quadrant of vision.
    */
   BFS_U3_27S5C_FE185C086HA_1(591.83671,
                              579.59183,
                              963.47389,
                              727.86785,
                              0.0211219,
                              -0.0059760,
                              0.0025230,
                              -0.0008246);

   private final double focalLengthXForUndistortion;
   private final double focalLengthYForUndistortion;
   private final double principalPointXForUndistortion;
   private final double principalPointYForUndistortion;
   private final double k1ForUndistortion;
   private final double k2ForUndistortion;
   private final double k3ForUndistortion;
   private final double k4ForUndistortion;

   BlackflyLensProperties(double focalLengthXForUndistortion,
                          double focalLengthYForUndistortion,
                          double principalPointXForUndistortion,
                          double principalPointYForUndistortion,
                          double k1ForUndistortion,
                          double k2ForUndistortion,
                          double k3ForUndistortion,
                          double k4ForUndistortion)
   {
      this.focalLengthXForUndistortion = focalLengthXForUndistortion;
      this.focalLengthYForUndistortion = focalLengthYForUndistortion;
      this.principalPointXForUndistortion = principalPointXForUndistortion;
      this.principalPointYForUndistortion = principalPointYForUndistortion;
      this.k1ForUndistortion = k1ForUndistortion;
      this.k2ForUndistortion = k2ForUndistortion;
      this.k3ForUndistortion = k3ForUndistortion;
      this.k4ForUndistortion = k4ForUndistortion;
   }

   public double getFocalLengthXForUndistortion()
   {
      return focalLengthXForUndistortion;
   }

   public double getFocalLengthYForUndistortion()
   {
      return focalLengthYForUndistortion;
   }

   public double getPrincipalPointXForUndistortion()
   {
      return principalPointXForUndistortion;
   }

   public double getPrincipalPointYForUndistortion()
   {
      return principalPointYForUndistortion;
   }

   public double getK1ForUndistortion()
   {
      return k1ForUndistortion;
   }

   public double getK2ForUndistortion()
   {
      return k2ForUndistortion;
   }

   public double getK3ForUndistortion()
   {
      return k3ForUndistortion;
   }

   public double getK4ForUndistortion()
   {
      return k4ForUndistortion;
   }

   public static BlackflyLensProperties forModel(BlackflyModelProperties blackflyModelProperties)
   {
      return switch (blackflyModelProperties)
      {
         case BFLY_U3_23S6C -> BFLY_U3_23S6C_FE185C086HA_1;
         default -> BFS_U3_27S5C_FE185C086HA_1;
      };
   }
}
