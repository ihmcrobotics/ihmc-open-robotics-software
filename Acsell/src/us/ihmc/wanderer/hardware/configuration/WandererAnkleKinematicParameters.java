package us.ihmc.wanderer.hardware.configuration;

import us.ihmc.acsell.hardware.configuration.AcsellAnkleKinematicParameters;

public final class WandererAnkleKinematicParameters implements AcsellAnkleKinematicParameters
{
   public WandererAnkleKinematicParameters()
   {
      throw new RuntimeException("TODO: Set correct values and remove constructor");
   }
   
   private static final double N = 6.0;

   //Wide bracket Spring linkage
   private static final double px[] = { -0.0000, 0.8791, -0.8791, -0.0149, 0.0149, -0.0000, -0.0604, 0.0439, -0.0439, 0.0604 };
   private static final double py[] = { 0.0009, 0.4611, 0.4611, 0.0283, 0.0283, 0.0005, -0.0681, 0.0575, 0.0575, -0.0681 };
   private static final double pM1[] = { 0.0086, 3.3934, 6.4627, -0.2408, -0.5256, 0.0963, 0.1911, 1.7066, 0.8220, 0.3498 };
   private static final double pM2[] = { 0.0086, -3.3934, 6.4627, -0.2408, -0.5256, -0.0963, -0.1911, 1.7066, -0.8220, 0.3498 };
   private static final double p_m_JitX[] = { 0.8827, -0.0349, -0.0045, -0.1410, 0.0098, -0.0209, -0.0950, 0.2628, -0.1591, 0.0121 };
   private static final double p_m_JitY[] = { 0.4619, 0.1085, -0.0576, -0.2250, 0.0451, 0.1548, -0.1208, 0.1888, -0.0829, 0.0507 };
   private static final double p_j_JitX[] = { 0.8834, -0.0204, -0.0408, -0.0321, -0.1963, -0.2211, -0.0744, -0.1504, 0.0837, 0.0519 };
   private static final double p_j_JitY[] = { 0.4637, 0.1027, 0.0539, -0.1251, -0.0482, -0.3714, -0.1127, -0.1165, -0.1780, 0.0591 };

   @Override
   public double[] getXParams()
   {
      return px;
   }

   @Override
   public double[] getYParams()
   {
      return py;
   }

   @Override
   public double[] getJITX_FromMotorParams()
   {
      return p_m_JitX;
   }

   @Override
   public double[] getJITY_FromMotorParams()
   {
      return p_m_JitY;
   }

   @Override
   public double[] getJITX_FromJointParams()
   {
      return p_j_JitX;
   }

   @Override
   public double[] getJITY_FromJointParams()
   {
      return p_j_JitY;
   }

   @Override
   public double[] getM1Params()
   {
      return pM1;
   }

   @Override
   public double[] getM2Params()
   {
      return pM2;
   }

   @Override
   public double getN()
   {
      return N;
   }

   @Override
   public boolean useJacobianComputedFromMotors()
   {
      return false;
   }

   @Override
   public boolean isJacobianFromJointAnglesComputationPerformed()
   {
      return true;
   }

   @Override
   public boolean isJacobianFromMotorAnglesComputationPerformed()
   {
      return true;
   }

}
