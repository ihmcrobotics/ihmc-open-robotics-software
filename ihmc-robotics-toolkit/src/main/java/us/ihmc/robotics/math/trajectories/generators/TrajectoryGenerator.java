package us.ihmc.robotics.math.trajectories.generators;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;

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

public class TrajectoryGenerator
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
            return (4 * waveAmp / wavePeriod) * Math.abs((((curSecTime - (wavePeriod / 4)) % wavePeriod) + wavePeriod) % wavePeriod - wavePeriod / 2) - waveAmp;
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

