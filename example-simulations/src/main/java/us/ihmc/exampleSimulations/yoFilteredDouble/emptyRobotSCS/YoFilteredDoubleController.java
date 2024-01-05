package us.ihmc.exampleSimulations.yoFilteredDouble;

import us.ihmc.robotics.math.filters.TransferFunctionDiscretizer;
import us.ihmc.robotics.math.filters.YoFilteredDouble;
import us.ihmc.robotics.math.trajectories.generators.TrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.TrajectoryGenerator.ChirpType;
import us.ihmc.robotics.math.trajectories.generators.TrajectoryGenerator.Trajectory;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoFilteredDoubleController implements RobotController {

	private YoRegistry registry;
	private double dt_ns;
	private YoDouble controllerTime;
	private int ticks = 0;
	
	// Filters 
	private final YoFilteredDouble Order1LP_Filtered_Var; 			// 1st Order Low-Pass Filtered Double
	private final YoFilteredDouble Order2LPButter_Filtered_Var;		// 2nd Order Low-Pass Butterworth Filtered Double
	private final YoFilteredDouble Notch_Filtered_Var; 				// Notch Filtered Double
	private final YoFilteredDouble MultiorderComplex_Filtered_Var; 	// More Complex Multi-order Filtered Double 
	private final YoFilteredDouble PID_Filtered_Var; 				// PID Controller Double
	private final YoFilteredDouble LeadLag_Filtered_Var; 			// Lead-Lag Controller Double
	private final YoFilteredDouble RefTraj_jump_Filtered_Var; 		// 2nd Order Low-Pass Filtered Double tracking reference demanding an instant jump.
	private final YoFilteredDouble RefTraj_nonjump_Filtered_Var; 	// 2nd Order Low-Pass Filtered Double tracking reference without instant jump.
	private final YoFilteredDouble RefTraj2_jump_Filtered_Var; 		// PID Controller Double tracking reference demanding an instant jump.
	private final YoFilteredDouble RefTraj2_nonjump_Filtered_Var; 	// PID Controller Double tracking reference without instant jump.
	private final TrajectoryGenerator chirp;
	private final TrajectoryGenerator chirp_lin;
	private final TrajectoryGenerator chirp_exp;
	private final TrajectoryGenerator sinusoid;
	private final YoDouble input;
	private final YoDouble chirpFreqHz;
	private final YoDouble input_lin;
	private final YoDouble chirpFreqHz_lin;
	private final YoDouble input_exp;
	private final YoDouble chirpFreqHz_exp;
	private final YoDouble sinusoid_input;
	
	// 1st Order Low-Pass Parameters
	private final double Tau_folp = 2*Math.PI*10;	
	
	// 2nd Order Low-Pass Butter Parameters
	private final double wc = 2*Math.PI*10.0;
	
	// Notch Filter Parameters
	private final double wn 	= 60*2*Math.PI;
	private final double Q 		= 5.0;
	
	// PID Parameters
	private final double Kp 	= 15.0;
	private final double Ki 	= 2.0;
	private final double Kd 	= 0.25;
	private final double Tau 	= 0.0035;
	
	// Lead-Lag Compensator Parameters
	private final double k 		= 10;
	private final double z 	 	= 2*Math.PI*1;
	private final double p 		= 2*Math.PI*10;
	
	public YoFilteredDoubleController(YoRegistry registry, double dt_ns) {
		this.registry = registry;
		this.dt_ns = dt_ns;
		this.controllerTime = new YoDouble("controllerTime", registry);
		
		// 2.0 Build Filter Objects.
		TransferFunctionDiscretizer FOLPFilter 						= 	new TransferFunctionDiscretizer(1.0, 				
																										new double[] {1.0},
																										new double[] { 1/Tau_folp, 1.0},
																										(1000000000.0 / ((double) dt_ns)));
		TransferFunctionDiscretizer LP_2nd_Order_Butter_Filter 		= 	new TransferFunctionDiscretizer(1.0, 				
																										new double[] {wc*wc},
																										new double[] { 1.0, Math.sqrt(2)*wc, wc*wc},
																										(1000000000.0 / ((double) dt_ns)));
		TransferFunctionDiscretizer Notch_Filter 					= 	new TransferFunctionDiscretizer(1.0,
																										new double[] { 1.0, 0.0, wn*wn},
																										new double[] { 1.0, wn/Q, wn*wn},
																										(1000000000.0 / ((double) dt_ns)));
		TransferFunctionDiscretizer MultiorderComplex_Filter 		= 	new TransferFunctionDiscretizer(1.0,
																										new double[] { 196.919515374308, 21033.790696845190, 427573.897431703983, 18317222.932339027524 },
																										new double[] { 1.000000000000, 382.156022138851, 60851.343857079330, 3875784.585037478711 },
																										(1000000000.0 / ((double) dt_ns)));
		TransferFunctionDiscretizer PID_Filter 						= 	new TransferFunctionDiscretizer(1.0, 				
																										new double[] {(Kp + Tau*Kd), (Tau*Kp + Ki), Ki*Tau},
																										new double[] { 1.0, Tau, 0.0},
																										(1000000000.0 / ((double) dt_ns)));
		TransferFunctionDiscretizer Lead_Lag_Compensator_Filter 	= 	new TransferFunctionDiscretizer(k, 				
																										new double[] {1.0, z},
																										new double[] { 1.0, p},
																										(1000000000.0 / ((double) dt_ns)));
		
		
		// 3.0 Create new filtered doubles based on the Filter objects.
		Order1LP_Filtered_Var 				= new YoFilteredDouble("Order1LPFilter_Var", registry, FOLPFilter, true);
		Order2LPButter_Filtered_Var 		= new YoFilteredDouble("Order2LPButter_Var", registry, LP_2nd_Order_Butter_Filter, true);
		Notch_Filtered_Var 					= new YoFilteredDouble("Notch_Var", registry, Notch_Filter, true);
		MultiorderComplex_Filtered_Var 		= new YoFilteredDouble("InvPlant_N_to_mA_Var", registry, MultiorderComplex_Filter, true);
		PID_Filtered_Var 					= new YoFilteredDouble("PID_Var", registry, PID_Filter, true);
		LeadLag_Filtered_Var 				= new YoFilteredDouble("LeadLag_Var", registry, Lead_Lag_Compensator_Filter, true);
		RefTraj_jump_Filtered_Var 			= new YoFilteredDouble("RefTraj_jump_Filtered_Var", registry, LP_2nd_Order_Butter_Filter, true);
		RefTraj_nonjump_Filtered_Var 		= new YoFilteredDouble("RefTraj_nonjump_Filtered_Var", registry, LP_2nd_Order_Butter_Filter, false); // Set false to avoid jump
		RefTraj2_jump_Filtered_Var 			= new YoFilteredDouble("RefTraj2_jump_Filtered_Var", registry, Lead_Lag_Compensator_Filter, true);
		RefTraj2_nonjump_Filtered_Var 		= new YoFilteredDouble("RefTraj2_nonjump_Filtered_Var", registry, Lead_Lag_Compensator_Filter, false); // Set false to avoid jump

		
		System.out.println("1st Order LP Input Coeffs: " + FOLPFilter.getInputCoefficients().toString());
		System.out.println("1st Order LP Output Coeffs: " + FOLPFilter.getOutputCoefficients().toString());
		
		System.out.println("2nd Order LP Butter Input Coeffs: " + LP_2nd_Order_Butter_Filter.getInputCoefficients().toString());
		System.out.println("2nd Order LP Butter Output Coeffs: " + LP_2nd_Order_Butter_Filter.getOutputCoefficients().toString());
		
		System.out.println("Notch Filter Input Coeffs: " + Notch_Filter.getInputCoefficients().toString());
		System.out.println("Notch Filter Output Coeffs: " + Notch_Filter.getOutputCoefficients().toString());
		
		System.out.println("Inverse Plant Input Coeffs: " + MultiorderComplex_Filter.getInputCoefficients().toString());
		System.out.println("Inverse Plant Output Coeffs: " + MultiorderComplex_Filter.getOutputCoefficients().toString());
		
		System.out.println("PID Controller Input Coeffs: " + PID_Filter.getInputCoefficients().toString());
		System.out.println("PID Controller Output Coeffs: " + PID_Filter.getOutputCoefficients().toString());
		
		System.out.println("Lead-Lag Controller Input Coeffs: " + Lead_Lag_Compensator_Filter.getInputCoefficients().toString());
		System.out.println("Lead-Lag Controller Output Coeffs: " + Lead_Lag_Compensator_Filter.getOutputCoefficients().toString());
		
		// 4.0 Initialize Input Chirp Trajectory.
		chirp 			= new TrajectoryGenerator("chirpSignal", registry, Trajectory.CHIRP, ChirpType.EXPONENTIAL, 100.0, 1.0, 0.00001, 500.0);
		chirp_lin 		= new TrajectoryGenerator("chirpSignal_lin", registry, Trajectory.CHIRP, ChirpType.LINEAR, 		10.0, 1.0, 0.01, 2.5);
		chirp_exp 		= new TrajectoryGenerator("chirpSignal_exp", registry, Trajectory.CHIRP, ChirpType.EXPONENTIAL, 10.0, 1.0, 0.01, 2.5);
		sinusoid 		= new TrajectoryGenerator("sinusoid", registry, Trajectory.SINE, 1.0, 100, 0.0, 0.0, 5.0);
		input_lin 		= new YoDouble("input_lin",registry);
		
		chirpFreqHz_lin = new YoDouble("chirpFreqHz_lin", registry);
		input_exp 		= new YoDouble("input_exp",registry);
		chirpFreqHz_exp = new YoDouble("chirpFreqHz_exp", registry);		
		input 			= new YoDouble("input",registry);
		chirpFreqHz 	= new YoDouble("chirpFreqHz", registry);
		sinusoid_input 	= new YoDouble("sinusoid_input", registry);
	}
	
	@Override
	public void initialize() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public YoRegistry getYoRegistry() {
		// TODO Auto-generated method stub
		return registry;
	}

	@Override
	public void doControl() {
		controllerTime.add(((double) dt_ns)/1000000000.0);
		input.set(chirp.updateTrajectory(controllerTime.getDoubleValue(), ((double) dt_ns)/1000000000.0));
		input_lin.set(chirp_lin.updateTrajectory(controllerTime.getDoubleValue(), ((double) dt_ns)/1000000000.0));
		input_exp.set(chirp_exp.updateTrajectory(controllerTime.getDoubleValue(), ((double) dt_ns)/1000000000.0));
		sinusoid_input.set(sinusoid.updateTrajectory(controllerTime.getDoubleValue(), ((double) dt_ns)/1000000000.0));
		chirpFreqHz_lin.set(chirp_lin.getFreqCheckRad()/(2*Math.PI));
		chirpFreqHz_exp.set(chirp_exp.getFreqCheckRad()/(2*Math.PI));
		Order1LP_Filtered_Var.set(input.getDoubleValue());
		Order2LPButter_Filtered_Var.set(input.getDoubleValue());
		Notch_Filtered_Var.set(input.getDoubleValue());
		MultiorderComplex_Filtered_Var.set(input.getDoubleValue());
		PID_Filtered_Var.set(input.getDoubleValue());
		LeadLag_Filtered_Var.set(input.getDoubleValue());
		RefTraj_jump_Filtered_Var.set(sinusoid_input.getDoubleValue());
		RefTraj_nonjump_Filtered_Var.set(sinusoid_input.getDoubleValue());
		RefTraj2_jump_Filtered_Var.set(sinusoid_input.getDoubleValue());
		RefTraj2_nonjump_Filtered_Var.set(sinusoid_input.getDoubleValue());
		chirpFreqHz.set(chirp.getFreqCheckRad()/(2*Math.PI));
		
	}

}
