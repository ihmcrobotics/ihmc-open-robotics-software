package us.ihmc.wanderer.hardware.configuration;

import us.ihmc.acsell.hardware.configuration.AcsellAnkleKinematicParameters;
import us.ihmc.acsell.hardware.configuration.AcsellAnklePhysicalParameters;

public final class WandererAnkleKinematicParameters implements AcsellAnkleKinematicParameters
{
   
   private static final double N = 6.0;

   //Wide bracket Spring linkage
   private static final double px[] = null;
   private static final double py[] = null;
   private static final double pM1[] = null;
   private static final double pM2[] = null;
   private static final double p_m_JitX[] = null;
   private static final double p_m_JitY[] = null;
   private static final double p_j_JitX[] = null;
   private static final double p_j_JitY[] = null;
   
   private static final double Px = 0.156401;
   private static final double Py = 0.055563;
   private static final double Pz = -0.026670;
   private static final double Lr = 0.320370;
   private static final double RyO = 0.039986;
   private static final double RyI = 0.044164;
   private static final double KzR = 0.317500;
   private static final double KzL = 0.317437;   
   
   private static final AcsellAnklePhysicalParameters rightAnkleRightParams = new AcsellAnklePhysicalParameters(
         new double[] {Px, -Py, Pz}, new double[] {0.124355, -RyO, 0.291713}, KzR, Lr);
   private static final AcsellAnklePhysicalParameters rightAnkleLeftParams = new AcsellAnklePhysicalParameters(
         new double[] {Px, Py, Pz}, new double[] {0.124392, RyI, 0.291893}, KzR, Lr);
   private static final AcsellAnklePhysicalParameters leftAnkleRightParams = new AcsellAnklePhysicalParameters(
         new double[] {Px, -Py, Pz}, new double[] {0.124406, -RyI, .291895}, KzL, Lr);
   private static final AcsellAnklePhysicalParameters leftAnkleLeftParams = new AcsellAnklePhysicalParameters(
         new double[] {Px, Py, Pz}, new double[] {0.124368, RyO, 0.291714}, KzL, Lr);

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
      return false;
   }
   
   @Override
   public AcsellAnklePhysicalParameters getRightAnkleRightParams()
   {
      return rightAnkleRightParams;
   }
   
   @Override
   public AcsellAnklePhysicalParameters getRightAnkleLeftParams()
   {
      return rightAnkleLeftParams;
   }
   
   @Override
   public AcsellAnklePhysicalParameters getLeftAnkleRightParams()
   {
      return leftAnkleRightParams;
   }
   
   @Override
   public AcsellAnklePhysicalParameters getLeftAnkleLeftParams()
   {
      return leftAnkleLeftParams;
   }

}

