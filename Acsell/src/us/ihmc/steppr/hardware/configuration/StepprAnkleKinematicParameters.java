package us.ihmc.steppr.hardware.configuration;

public final class StepprAnkleKinematicParameters {
	
	   private static final double px[] = { 0.000000000011833, 1.269023941053808, -1.269023941048548, 0.020516020869627, -0.020516021471959, 0.000000000606667,
	         0.115495040111334, -0.523312217421704, 0.523312217621650, -0.115495040305941 };

	   private static final double py[] = { 0.000022001174462, 0.463732921959109, 0.463732921783293, -0.048394601071517, -0.048394601065684, 0.111755077611979,
	         -0.075022109801143, 0.057936370829754, 0.057936363252024, -0.075022107299457 };

	   private static final double p_m_JitX[] = { 1.272997427777619, 0.036001701491556, 0.001270562468774, 0.411410640321290, 0.628599891388498, -1.233612302811653,
	         -0.128100680179965, 0.209457856510320, 0.014033597275913, -0.080071351885635 };

	   private static final double p_m_JitY[] = { 0.465442714987602, -0.079404498198185, 0.091522714637596, -0.239129834635615, 0.040327656123728, 0.141924052147264,
	         -0.130148712431276, 0.342409832258836, -0.315199115178924, 0.111764662233772 };
	   
	   private static final double pM1[] = { 0.001637333497514, 2.363024415727514, 6.448426531047804, 0.209647991746190, -0.132578850323121, -0.102263276506052,
	         -0.223803048871049, 0.764689039287297, 0.459191580065332, 0.349362854084994 };
	   
	   private static final double pM2[] = { 0.001637333371246, -2.363024413970044, 6.448426532362931, 0.209647993650533, -0.132578850320880, 0.102263276373644,
	         0.223803036060187, 0.764689036580238, -0.459191581057666, 0.349362851648649 };
	   
	   private static final double p_j_JitX[] = { 1.272997427777619, 0.036001701491556, 0.001270562468774, 0.411410640321290, 0.628599891388498, -1.233612302811653,
	         -0.128100680179965, 0.209457856510320, 0.014033597275913, -0.080071351885635 };

	   private static final double p_j_JitY[] = { 0.465442714987602, -0.079404498198185, 0.091522714637596, -0.239129834635615, 0.040327656123728, 0.141924052147264,
	         -0.130148712431276, 0.342409832258836, -0.315199115178924, 0.111764662233772 };

	   private static final double N = 6.0; 
	   
	   public static double[] getXParams()
	   {
		   return px;
	   }
	   
	   public static double[] getYParams()
	   {
		   return py;
	   }
	   
	   public static double[] getJITX_FromMotorParams()
	   {
		   return p_m_JitX;
	   }
	   
	   public static double[] getJITY_FromMotorParams()
	   {
		   return p_m_JitY;
	   }
	   
	   public static double[] getJITX_FromJointParams()
	   {
		   return p_j_JitX;
	   }
	   
	   public static double[] getJITY_FromJointParams()
	   {
		   return p_j_JitY;
	   }
	   
	   public static double[] getM1Params()
	   {
		   return pM1;
	   }
	   
	   public static double[] getM2Params()
	   {
		   return pM2;
	   }
	   
	   public static double getN()
	   {
		   return N;
	   }
	   
}
