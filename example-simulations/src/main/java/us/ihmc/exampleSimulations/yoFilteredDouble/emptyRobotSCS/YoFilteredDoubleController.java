package us.ihmc.exampleSimulations.yoFilteredDouble.emptyRobotSCS;

import us.ihmc.exampleSimulations.yoFilteredDouble.emptyRobotSCS.YoFilteredDoubleController.TrajectoryGenerator.ChirpType;
import us.ihmc.exampleSimulations.yoFilteredDouble.emptyRobotSCS.YoFilteredDoubleController.TrajectoryGenerator.Trajectory;
import us.ihmc.robotics.math.filters.ContinuousTransferFunction;
import us.ihmc.robotics.math.filters.TransferFunctionDiscretizer;
import us.ihmc.robotics.math.filters.YoFilteredDouble;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;

public class YoFilteredDoubleController implements RobotController
{
   private YoRegistry registry;
   private double dt_ns;
   private YoDouble controllerTime;
   private int ticks = 0;

   // Filters
   private final YoFilteredDouble Order1LP_Filtered_Var;         // 1st Order Low-Pass Filtered Double
   private final YoFilteredDouble Order2LPButter_Filtered_Var;      // 2nd Order Low-Pass Butterworth Filtered Double
   private final YoFilteredDouble Notch_Filtered_Var;            // Notch Filtered Double
   private final YoFilteredDouble MultiorderComplex_Filtered_Var;   // More Complex Multi-order Filtered Double
   private final YoFilteredDouble PID_Filtered_Var;            // PID Controller Double
   private final YoFilteredDouble LeadLag_Filtered_Var;         // Lead-Lag Controller Double
   private final YoFilteredDouble RefTraj_jump_Filtered_Var;      // 2nd Order Low-Pass Filtered Double tracking reference demanding an instant jump.
   private final YoFilteredDouble RefTraj_nonjump_Filtered_Var;   // 2nd Order Low-Pass Filtered Double tracking reference without instant jump.
   private final YoFilteredDouble RefTraj2_jump_Filtered_Var;      // PID Controller Double tracking reference demanding an instant jump.
   private final YoFilteredDouble RefTraj2_nonjump_Filtered_Var;   // PID Controller Double tracking reference without instant jump.
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
   private final double Tau_folp = 2 * Math.PI * 10;

   // 2nd Order Low-Pass Butter Parameters
   private final double wc = 2 * Math.PI * 10.0;

   // Notch Filter Parameters
   private final double wn = 60 * 2 * Math.PI;
   private final double Q = 1.0;

   // PID Parameters
   private final double Kp = 15.0;
   private final double Ki = 2.0;
   private final double Kd = 0.25;
   private final double Tau = 0.0035;

   // Lead-Lag Compensator Parameters
   private final double k = 10;
   private final double z = 2 * Math.PI * 1;
   private final double p = 2 * Math.PI * 10;

   public YoFilteredDoubleController(YoRegistry registry, double dt_ns)
   {
      this.registry = registry;
      this.dt_ns = dt_ns;
      this.controllerTime = new YoDouble("controllerTime", registry);

      // 2.0 Build Filter Objects.
      ContinuousTransferFunction tf1 = new ContinuousTransferFunction("First Order Low-Pass Filter", 1.0, new double[] {1.0}, new double[] {1 / Tau_folp, 1.0});
      TransferFunctionDiscretizer FOLPFilter = new TransferFunctionDiscretizer(tf1, 1000.0);

      ContinuousTransferFunction tf2 = new ContinuousTransferFunction("Second Order Low-Pass Butterworth Filter",
                                                                      1.0,
                                                                      new double[] {wc * wc},
                                                                      new double[] {1.0, Math.sqrt(2) * wc, wc * wc});

      TransferFunctionDiscretizer LP_2nd_Order_Butter_Filter = new TransferFunctionDiscretizer(tf2, 1000.0);

      ContinuousTransferFunction tf3 = new ContinuousTransferFunction("Second Order Notch Filter",
                                                                      1.0,
                                                                      new double[] {1.0, 0.0, wn * wn},
                                                                      new double[] {1.0, wn / Q, wn * wn});

      TransferFunctionDiscretizer Notch_Filter = new TransferFunctionDiscretizer(tf3, 1000.0);

      ContinuousTransferFunction tf4 = new ContinuousTransferFunction("Complex Multi-Order Filter",
                                                                      1.0,
                                                                      new double[] {196.919515374308,
                                                                                    21033.790696845190,
                                                                                    427573.897431703983,
                                                                                    18317222.932339027524},
                                                                      new double[] {1.000000000000,
                                                                                    382.156022138851,
                                                                                    60851.343857079330,
                                                                                    3875784.585037478711});
      TransferFunctionDiscretizer MultiorderComplex_Filter = new TransferFunctionDiscretizer(tf4, 1000.0);

      ContinuousTransferFunction tf5 = new ContinuousTransferFunction("PID Controller",
                                                                      1.0,
                                                                      new double[] {(Kp + Tau * Kd), (Tau * Kp + Ki), Ki * Tau},
                                                                      new double[] {1.0, Tau, 0.0});

      TransferFunctionDiscretizer PID_Filter = new TransferFunctionDiscretizer(tf5, 1000.0);

      ContinuousTransferFunction tf6 = new ContinuousTransferFunction("Lead-Lag Controller", k, new double[] {1.0, z}, new double[] {1.0, p});

      TransferFunctionDiscretizer Lead_Lag_Compensator_Filter = new TransferFunctionDiscretizer(tf6, 1000.0);

      // 3.0 Create new filtered doubles based on the Filter objects.
      Order1LP_Filtered_Var = new YoFilteredDouble("Order1LPFilter_Var", registry, FOLPFilter, true);
      Order2LPButter_Filtered_Var = new YoFilteredDouble("Order2LPButter_Var", registry, LP_2nd_Order_Butter_Filter, true);
      Notch_Filtered_Var = new YoFilteredDouble("Notch_Var", registry, Notch_Filter, true);
      MultiorderComplex_Filtered_Var = new YoFilteredDouble("InvPlant_N_to_mA_Var", registry, MultiorderComplex_Filter, true);
      PID_Filtered_Var = new YoFilteredDouble("PID_Var", registry, PID_Filter, true);
      LeadLag_Filtered_Var = new YoFilteredDouble("LeadLag_Var", registry, Lead_Lag_Compensator_Filter, true);
      RefTraj_jump_Filtered_Var = new YoFilteredDouble("RefTraj_jump_Filtered_Var", registry, LP_2nd_Order_Butter_Filter, true);
      RefTraj_nonjump_Filtered_Var = new YoFilteredDouble("RefTraj_nonjump_Filtered_Var",
                                                          registry,
                                                          LP_2nd_Order_Butter_Filter,
                                                          false); // Set false to avoid jump
      RefTraj2_jump_Filtered_Var = new YoFilteredDouble("RefTraj2_jump_Filtered_Var", registry, Lead_Lag_Compensator_Filter, true);
      RefTraj2_nonjump_Filtered_Var = new YoFilteredDouble("RefTraj2_nonjump_Filtered_Var",
                                                           registry,
                                                           Lead_Lag_Compensator_Filter,
                                                           false); // Set false to avoid jump

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
      chirp = new TrajectoryGenerator("chirpSignal", registry, Trajectory.CHIRP, ChirpType.EXPONENTIAL, 20.0, 1.0, 0.00001, 100.0);
      chirp_lin = new TrajectoryGenerator("chirpSignal_lin", registry, Trajectory.CHIRP, ChirpType.LINEAR, 20.0, 1.0, 0.00001, 2.5);
      chirp_exp = new TrajectoryGenerator("chirpSignal_exp", registry, Trajectory.CHIRP, ChirpType.EXPONENTIAL, 20.0, 1.0, 0.00001, 2.5);
      sinusoid = new TrajectoryGenerator("sinusoid", registry, Trajectory.SINE, 1.0, 100, 0.0, 0.0, 5.0);
      input_lin = new YoDouble("input_lin", registry);

      chirpFreqHz_lin = new YoDouble("chirpFreqHz_lin", registry);
      input_exp = new YoDouble("input_exp", registry);
      chirpFreqHz_exp = new YoDouble("chirpFreqHz_exp", registry);
      input = new YoDouble("input", registry);
      chirpFreqHz = new YoDouble("chirpFreqHz", registry);
      sinusoid_input = new YoDouble("sinusoid_input", registry);
   }

   @Override
   public void initialize()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public YoRegistry getYoRegistry()
   {
      // TODO Auto-generated method stub
      return registry;
   }

   @Override
   public void doControl()
   {
      controllerTime.add(((double) dt_ns) / 1000000000.0);
      input.set(chirp.updateTrajectory(controllerTime.getDoubleValue(), ((double) dt_ns) / 1000000000.0));
      input_lin.set(chirp_lin.updateTrajectory(controllerTime.getDoubleValue(), ((double) dt_ns) / 1000000000.0));
      input_exp.set(chirp_exp.updateTrajectory(controllerTime.getDoubleValue(), ((double) dt_ns) / 1000000000.0));
      sinusoid_input.set(sinusoid.updateTrajectory(controllerTime.getDoubleValue(), ((double) dt_ns) / 1000000000.0));
      chirpFreqHz_lin.set(chirp_lin.getFreqCheckRad() / (2 * Math.PI));
      chirpFreqHz_exp.set(chirp_exp.getFreqCheckRad() / (2 * Math.PI));
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
      chirpFreqHz.set(chirp.getFreqCheckRad() / (2 * Math.PI));
   }

   /**
    * Class is designed for quickly generating several different trajectory types. A majority of the classes are self-explanatory.
    * The Triangle Waveform comes from https://en.wikipedia.org/wiki/Triangle_wave, denoting an alternative equation because the mod
    * operator is not the remainder operator.
    * The Multi-Square Waveform will generate a series of square waves in order repeating forever. It can be initialized using:
    * DesiredTrajectory = new TrajectoryGenerator("OpenLoop", registry, Trajectory.MULTI_SQUARE, 2000.0, 0.2, new ArrayList<Double>(Arrays.asList(1000.0, 0.0,
    * 2000.0, 0.0, 3000.0, 0.0)));
    * The Chirp signal is a sinusoid with a consistent amplitude, but is increasing in frequency. It is commonly used for System ID,
    * and was used for identifying the transfer function of the actuator. The equation was given by Dr. Southward from VT in the ME Dept.
    * There are two options for chirp signals: linear or exponential. This refers to the rate at which the frequency is increasing and
    * this choice depends on which frequencies you care about.
    *
    * @author Connor Herron
    */

   public static class TrajectoryGenerator
   {
      private String name;
      private double totalTime;
      private double minFreqRad;
      private double maxFreqRad;
      private double prevCos;
      private double prevSin;
      private double chirpSlope;
      private double chirpAmplitude;
      private double curFreqRad;
      private double expoCoeff;
      private YoDouble freqCheckRad;
      private YoRegistry registry;
      private ArrayList<Double> multiSquareList = new ArrayList<>();

      private double waveAmp;
      private double waveFreq;
      private double rampSlope;
      private double phaseOffset;
      private double amplitudeOffset;
      private double wavePeriod;

      private int initFlag = 0;

      public enum Trajectory
      {
         STEP, RAMP, SINE, TRIANGLE, SAWTOOTH, SQUARE, MULTI_SQUARE, CHIRP
      }

      public enum ChirpType
      {
         LINEAR, EXPONENTIAL
      }

      private Trajectory myTrajectory;
      private ChirpType myChirpType;

      public TrajectoryGenerator(String name, YoRegistry registry, Trajectory desiredWaveform, double waveAmp)
      {
         this(name, registry, desiredWaveform, waveAmp, 0.0);
      }

      public TrajectoryGenerator(String name, YoRegistry registry, Trajectory desiredWaveform, double waveAmp, double waveFreq)
      {
         this(name, registry, desiredWaveform, waveAmp, waveFreq, 0.0);
      }

      public TrajectoryGenerator(String name, YoRegistry registry, Trajectory desiredWaveform, double waveAmp, double waveFreq, double rampSlope)
      {
         this(name, registry, desiredWaveform, waveAmp, waveFreq, rampSlope, 0.0, 0.0);
      }

      public TrajectoryGenerator(String name,
                                 YoRegistry registry,
                                 Trajectory desiredWaveform,
                                 double waveAmp,
                                 double waveFreq,
                                 double rampSlope,
                                 double phaseOffset,
                                 double amplitudeOffset)
      {
         this.name = name;
         this.registry = registry;
         this.myTrajectory = desiredWaveform;
         this.waveAmp = waveAmp;
         this.waveFreq = waveFreq;
         this.wavePeriod = 1 / waveFreq;
         this.rampSlope = rampSlope;
         this.phaseOffset = phaseOffset;
         this.amplitudeOffset = amplitudeOffset;
      }

      public TrajectoryGenerator(String name, YoRegistry registry, Trajectory desiredWaveform, double waveAmp, double waveFreq, ArrayList<Double> multiSquare)
      {
         this.name = name;
         this.registry = registry;
         this.myTrajectory = desiredWaveform;
         this.waveAmp = waveAmp;
         this.waveFreq = waveFreq;
         this.wavePeriod = wavePeriod;
         this.multiSquareList = multiSquare;
      }

      // For running chirp signals
      public TrajectoryGenerator(String name,
                                 YoRegistry registry,
                                 Trajectory desiredWaveform,
                                 ChirpType desiredChirpType,
                                 double totalTime,
                                 double amplitude,
                                 double minFreqHz,
                                 double maxFreqHz)
      {
         this.name = name;
         this.registry = registry;
         this.myTrajectory = desiredWaveform;
         this.myChirpType = desiredChirpType;

         freqCheckRad = new YoDouble(name + "freqCheckRad", registry);

         initializeChirpSignal(totalTime, amplitude, minFreqHz, maxFreqHz);
      }

      public void initializeChirpSignal(double totalTime, double amplitude, double minFreqHz, double maxFreqHz)
      {
         this.totalTime = totalTime;
         this.minFreqRad = 2 * Math.PI * minFreqHz;
         this.maxFreqRad = 2 * Math.PI * maxFreqHz;
         this.chirpAmplitude = amplitude;

         // Calculate the slope of the chirp signal.
         this.chirpSlope = (this.maxFreqRad - this.minFreqRad) / (totalTime);

         // Initialize coefficient for the exponential function
         this.expoCoeff = Math.log10(maxFreqRad / minFreqRad) / totalTime;

         // initialize prevCos and prevSin
         this.prevCos = 1.0;
         this.prevSin = 0.0;
      }

      private double alpha;
      private double beta;
      private double cosNew;
      private double sinNew;

      /**
       * Method calculates the open loop current command (mA) to send for a linear sinusoidal chirp signal. This method is initialized
       * with the initializeOLChirpSignal method above.
       *
       * @param t_sec real time seconds.
       * @param dT    real time control dT
       * @return commanded motor current (mA)
       */
      public double runChirpSignal(double t_sec, double dT)
      {
         // Calculate current Freq (Rad)
         switch (myChirpType)
         {
            case LINEAR:
               this.curFreqRad = (this.chirpSlope * t_sec + this.minFreqRad);
               break;
            case EXPONENTIAL:
               this.curFreqRad = this.minFreqRad * Math.pow(10, this.expoCoeff * t_sec);
               break;
         }

         // Calculate new sinusoid value using double angle identity formula.
         alpha = Math.cos(this.curFreqRad * dT);
         beta = Math.sin(this.curFreqRad * dT);

         cosNew = alpha * this.prevCos - beta * this.prevSin;
         sinNew = beta * this.prevCos + alpha * this.prevSin;

         this.prevCos = cosNew;
         this.prevSin = sinNew;

         if (t_sec > this.totalTime)
         {
            freqCheckRad.set(0.0);
            return 0.0;
         }
         else
         {
            freqCheckRad.set(this.curFreqRad);
            return this.chirpAmplitude * sinNew;
         }
      }

      // Run method during loop.
      public double updateTrajectory(double curSecTime, double dT)
      {
         switch (myTrajectory)
         {
            case STEP:
               return waveAmp;
            case RAMP:
               return rampSlope * curSecTime;
            case SINE:
               return waveAmp * Math.sin(2 * Math.PI * waveFreq * curSecTime + phaseOffset) + amplitudeOffset;
            case TRIANGLE:
               return (4 * waveAmp / wavePeriod) * Math.abs((((curSecTime - (wavePeriod / 4)) % wavePeriod) + wavePeriod) % wavePeriod - wavePeriod / 2)
                      - waveAmp;
            case SAWTOOTH:
               return waveAmp * waveFreq * Math.abs(curSecTime % (1 / waveFreq));
            case SQUARE:
               return waveAmp * Math.signum(Math.sin(2 * Math.PI * waveFreq * curSecTime + phaseOffset)) + amplitudeOffset;
            case MULTI_SQUARE:
               return waveAmp = multiSquareList.get(Math.floorMod((int) (curSecTime * waveFreq), multiSquareList.size()));
            case CHIRP:
               return runChirpSignal(curSecTime, dT);
            default:
               return 0.0;
         }
      }

      // Wave Frequency is defined as [rad/s]
      public double updateTrajectoryDot(Trajectory Desired_Trajectory, double curSecTime, double waveAmp, double waveFreq, double phaseOffset)
      {
         if (Desired_Trajectory == Trajectory.SINE)
         {
            return 2 * Math.PI * waveFreq * waveAmp * Math.cos(2 * Math.PI * waveFreq * curSecTime + phaseOffset);
         }
         else
         {
            return 0;
         }
      }

      public double getFreqCheckRad()
      {
         return freqCheckRad.getDoubleValue();
      }
   }
}
