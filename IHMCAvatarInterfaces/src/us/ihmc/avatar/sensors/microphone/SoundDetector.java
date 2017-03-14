package us.ihmc.avatar.sensors.microphone;

import java.io.ByteArrayOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

import javax.sound.sampled.AudioFormat;
import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.DataLine;
import javax.sound.sampled.LineUnavailableException;
import javax.sound.sampled.Mixer;
import javax.sound.sampled.TargetDataLine;
import javax.swing.JLabel;

import org.jtransforms.fft.DoubleFFT_1D;

import us.ihmc.commons.Conversions;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DrillDetectionPacket;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;

/**
 * 
 * <p>Title: SoundDetector</p>
 *
 * <p>Description: Detects a distinct sound by searching for a characteristic peak in FFT magnitude data of sound data from the Atlas Chest Webcam microphone around a given frequency</p>
 * 
 * 
 * @author Will
 *
 *
 */
public class SoundDetector implements Runnable
{
   private static final float sampleRate = 44100; //Hz
   private static final int sampleSizeInBits = 16;
   private static final int channels = 1;
   private static final boolean signed = true;
   private static final boolean bigEndian = false;
   private static final AudioFormat format = new AudioFormat(sampleRate, sampleSizeInBits, channels, signed, bigEndian);
   private static final int bufferSizeDesired = 16384 * 8;
   private static final int checkForRotozipFrequencyHz = 2; //Hz
   private JLabel statusIndicator = null;
   private final ArrayList<Double[]> peakCriteriaArrayList = new ArrayList<Double[]>();
   private static final String COMMA_DELIMITER = ",";
   private static final String NEW_LINE_SEPARATOR = "\n";
   public static AudioInputStream audioInputStream = null;
   Thread thread = null;
   TargetDataLine line = null;
   String errStr;
   double duration, seconds;
   private PacketCommunicator client = null;
   private boolean runningTheRealSchebang = false;

   public static void main(String[] args)
   {

      SoundDetector soundDetector = new SoundDetector();
      Thread processThread = new Thread(soundDetector);
      processThread.setName("main");
      processThread.start();

   }

   //Detection parameters
   private void setDetectionParams()
   {
      peakCriteriaArrayList.add(new Double[] { 420.0, 480.0, 250.0, 550.0, 9.0 });
      //      peakCriteriaArrayList.add(new Double[] { 2150.0, 2180.0, 2120.0, 2210.0, 5.0 });
      peakCriteriaArrayList.add(new Double[] { 5000.0, 6000.0, 4500.0, 6500.0, 7.0 });
   }

   /**
    * Meant for GUI Testing. Pass a JLabel for indcation that the drill is on
    * @param statusIndicator
    */
   public SoundDetector(JLabel statusIndicator)
   {
      setDetectionParams();
      runningTheRealSchebang = false; //We're probably just running the test GUI for tuning.
      this.statusIndicator = statusIndicator;
   }

   /**
    * Meant for background process to run on Atlas
    */
   public SoundDetector()
   {

      setDetectionParams();
      runningTheRealSchebang = true; //We're sending packets to the network processor. **** just got real.
      String networkManagerHost = NetworkParameters.getHost(NetworkParameterKeys.networkManager);
      client = PacketCommunicator.createTCPPacketCommunicatorClient(networkManagerHost, NetworkPorts.DRILL_DETECTOR,
            new IHMCCommunicationKryoNetClassList());
      try
      {
         System.out.println(getClass().getSimpleName() + ": Connecting to " + networkManagerHost);
         client.connect();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

   }

   private void shutDown(String errorMessage)
   {
      System.err.println(getClass().getSimpleName() + errorMessage);
      thread = null;
   }

   public void run()
   {

      thread = new Thread("run");
      thread.start();

      DataLine.Info info = new DataLine.Info(TargetDataLine.class, format);

      if (!AudioSystem.isLineSupported(info))
      {
         shutDown(": Audio line not supported");
      }

      try
      {
         line = (TargetDataLine) AudioSystem.getLine(info);
         line.open(format, bufferSizeDesired); //line.getBufferSize() / 8;
      }
      catch (LineUnavailableException ex)
      {
         shutDown("Unable to open the line: " + ex);
         System.out.println("Mixers available:");
         for(Mixer.Info infos : AudioSystem.getMixerInfo()){
            System.out.println(infos.toString());
         }
         return;
      }
      catch (SecurityException ex)
      {
         shutDown(ex.toString());
         return;
      }
      catch (Exception ex)
      {
         shutDown(ex.toString());
         return;
      }

      line.start();
      int frameSizeInBytes = format.getFrameSize();
      int bufferLengthInFrames = bufferSizeDesired / 8; //line.getBufferSize() / 8;
      int bufferLengthInBytes = bufferLengthInFrames * frameSizeInBytes;
      byte[] data = new byte[bufferLengthInBytes];
      int numBytesRead;

      while (thread != null)
      {
         if (runningTheRealSchebang)
         {

            if (!client.isConnected())
            {
               System.out.println("SoundDetector not connected to NP!");
               continue;
            }

         }
         ByteArrayOutputStream out = new ByteArrayOutputStream();
         if ((numBytesRead = line.read(data, 0, bufferLengthInBytes)) == -1)
         {
            break;
         }
         out.write(data, 0, numBytesRead);
         try
         {
            out.flush();
         }
         catch (IOException e)
         {
            shutDown("ByteArrayOutputStream.flush IOException");
         }

         byte audioBytes[] = out.toByteArray();

         if (runningTheRealSchebang)
         {

            DrillDetectionPacket drillDetectionPacket = new DrillDetectionPacket();
            drillDetectionPacket.isDrillOn = detectDrillFrequency(audioBytes);
            if(drillDetectionPacket.isDrillOn){
               System.out.println("isDrillOn = true");
            }
            client.send(drillDetectionPacket);
         }
         else
         {

            if (detectDrillFrequency(audioBytes) & statusIndicator != null)
            {
               statusIndicator.setText("ROTOZIP DETECTED!!!");
            }
            else if (statusIndicator != null)
            {
               statusIndicator.setText("Listening...");
            }

         }

         try
         {
            Thread.sleep(1000 / checkForRotozipFrequencyHz);
         }
         catch (InterruptedException e)
         {
            //            e.printStackTrace();
         }

         try
         {
            out.flush();
            out.close();
         }
         catch (IOException ex)
         {
            ex.printStackTrace();
         }
      }

      line.stop();
      line.close();
      line = null;

   }

   private boolean detectDrillFrequency(byte[] audioBytes)
   {
      boolean isDrillOn = false;
      boolean arePeaksThere = false;
      FileWriter fileWriter = null;
      FileWriter fileWriterBode = null;
      if (audioBytes != null)
      {
         int[] audioData = null;
         if (format.getSampleSizeInBits() == 16)
         {
            int nlengthInSamples = audioBytes.length / 2;
            audioData = new int[nlengthInSamples];

            //Little-Endian Code
            for (int i = 0; i < nlengthInSamples; i++)
            {
               /* First byte is LSB (low order) */
               int LSB = (int) audioBytes[2 * i];
               /* Second byte is MSB (high order) */
               int MSB = (int) audioBytes[2 * i + 1];
               audioData[i] = MSB << 8 | (255 & LSB);
            }

            if (!runningTheRealSchebang)
            {
               //Write buffer data to a CSV file
               //            /**
               try
               {
                  fileWriter = new FileWriter("buffercsv.csv");

                  for (int intToWrite : audioData)
                  {

                     fileWriter.append(intToWrite + COMMA_DELIMITER);
                     fileWriter.append(NEW_LINE_SEPARATOR);

                  }
                  fileWriter.flush();
                  fileWriter.close();
               }
               catch (IOException e)
               {
                  e.printStackTrace();
               }
            }
            //            **/

            double[] input = new double[audioData.length];
            double[] time = new double[audioData.length];
            int i = 0;
            for (int intToCast : audioData)
            {
               input[i] = (double) intToCast;
               time[i] = i * 1 / format.getSampleRate();
               i++;
            }

            double[][] fftData = computeFreqMagPhase(time, input);
            double[] frequency = fftData[0];
            double[] magnitude = Conversions.convertMagnitudeToDecibels(fftData[1]);
            //            double[] phase = fftData[2];

            if (!runningTheRealSchebang)
            {
               //Write frequency and magnitude data to a CSV file
               try
               {
                  fileWriterBode = new FileWriter("freq_mag.csv");

                  for (int i1 = 0; i1 < frequency.length; i1++)
                  {

                     fileWriterBode.append(frequency[i1] + ", " + magnitude[i1] + ",");
                     fileWriterBode.append(NEW_LINE_SEPARATOR);

                  }
                  fileWriterBode.flush();
                  fileWriterBode.close();
               }
               catch (IOException e)
               {
                  e.printStackTrace();
               }
               
               AudioFFTPlotter plot = new AudioFFTPlotter(frequency, magnitude, "Magnitude Plot", "(Hz)", "(dB)");
               plot.packAndDisplayFrame(0, 0);
               
            }

            for (Double[] peakCriteria : peakCriteriaArrayList)
            {

               if (!scanBandForPeak(frequency, magnitude, peakCriteria))
               {
                  arePeaksThere = false;
                  isDrillOn = false;
                  break;
               }
               else
               {
                  arePeaksThere = true;
                  isDrillOn = true;
               }
            }

            if (isFourHundredPeakLoudEnough(frequency, magnitude, peakCriteriaArrayList) & arePeaksThere)
            {
               isDrillOn = true;
            }
            else
            {
               isDrillOn = false;
            }

         }
      }
      return isDrillOn;
   }

   public static double[][] computeFreqMagPhase(double[] time, double[] data)
   {
      int n = time.length;
      if (data.length != n)
         throw new RuntimeException("input.length != n");

      DoubleFFT_1D fft = new DoubleFFT_1D(n);

      double T = time[n - 1] - time[0];

      double fftInput[] = copyDoubleArray(data);

      fft.realForward(fftInput);

      double[] magnitude = extractMagnitude(fftInput);
      double[] phase = extractPhaseRadians(fftInput);
      double[] frequency = new double[n / 2];
      for (int i = 0; i < n / 2; i++)
      {
         frequency[i] = (i) / T;
      }

      return new double[][] { frequency, magnitude, phase };
   }

   private static double[] copyDoubleArray(double[] in)
   {
      double[] ret = new double[in.length];

      for (int i = 0; i < in.length; i++)
      {
         ret[i] = in[i];
      }

      return ret;
   }

   private static double[] extractMagnitude(double[] fftInput)
   {
      int n = fftInput.length;

      //    if (n % 2 != 0)
      //        throw new RuntimeException("Must be even length!");

      double[] magnitude = new double[n / 2];

      for (int k = 0; k < n / 2; k++)
      {
         double real = fftInput[2 * k];
         double imag = fftInput[2 * k + 1];

         if (k == 0)
            imag = 0; // This is due to storing a[1] = Re[n/2] when n is even and = Im[n/2-1] when n is odd, as per the fft specs. Just ignore that term for now...

         double mag = Math.sqrt(real * real + imag * imag);
         magnitude[k] = mag;

         //       magnitude[k] = 20.0 * Math.log10(mag);
      }

      return magnitude;
   }

   private static double[] extractPhaseRadians(double[] fftInput)
   {
      int n = fftInput.length;

      //    if (n % 2 != 0)
      //        throw new RuntimeException("Must be even length!");

      double[] phase = new double[n / 2];

      for (int k = 0; k < n / 2; k++)
      {
         double real = fftInput[2 * k];
         double imag = fftInput[2 * k + 1];

         if (k == 0)
            imag = 0; // This is due to storing a[1] = Re[n/2] when n is even and = Im[n/2-1] when n is odd, as per the fft specs. Just ignore that term for now...

         phase[k] = Math.atan2(imag, real);
      }

      return phase;
   }

   /**
    * 
    * A method to help avoid false positives from peaks that meet scanForPeak criteria but arent loud enough to be a rotozip.
    * 
    * @param frequency
    * @param magnitude
    * @return
    * 
    * A boolean that is true if the sound and at around 430Hz is loud enough. 430Hz hardcoded in here.
    */
   private boolean isFourHundredPeakLoudEnough(double[] frequency, double[] magnitude, ArrayList<Double[]> peakCriteriaArrayList)
   {
      boolean peakLoudEnough = false;
      double peakFourThirty = -100.0;
      double lowerBoundFreq = peakCriteriaArrayList.get(0)[0];
      double upperBoundFreq = peakCriteriaArrayList.get(0)[1];
      int lowerBoundIndex = 0;
      int upperBoundIndex = 0;
      for (int index = 0; index < frequency.length; index++)
      {
         if (frequency[index] < lowerBoundFreq)
         {
            lowerBoundIndex = index;
         }
         if (frequency[index] < upperBoundFreq)
         {
            upperBoundIndex = index;
         }
      }
      for (int index = lowerBoundIndex; index < upperBoundIndex; index++)
      {
         peakFourThirty = (magnitude[index] > peakFourThirty ? magnitude[index] : peakFourThirty);
      }

      if (peakFourThirty > 135.0)
      {
         peakLoudEnough = true;
      }

      return peakLoudEnough;
   }

   /**
    * 
    * Detects peaks in the given magnitude data at frequency ranges given in the peakCriteria (see below)
    * 
    * @param frequency
    * @param magnitude
    * @param peakCriteria[0] = dominantBandLowerBound
    * @param peakCriteria[1] = dominantBandUpperBound
    * @param peakCriteria[2] = relevantBandLowerBound
    * @param peakCriteria[3] = relevantBandUpperBand
    * @param peakCriteria[5] = decibelsToTrip
    * @param peakCriteria
    * 
    * The relevant frequency band is the range of frequencies that are used for comparison to distinguish a peak. 
    * The magnitude data is averaged across this range with the exception of the dominant frequency band magnitudes.
    * 
    * The dominant frequency band is the range of frequencies were the peak in magnitude could show up in. This band must be within the relevant band.
    * 
    * The decibelsToTrip is the minimum difference in magnitude the peak must have over the average magnitude to be considered a significant enough peak
    * to trip detection
    * 
    * @return
    * 
    * A boolean that is true if there is a peak that meets the peak criteria.
    */
   private boolean scanBandForPeak(double[] frequency, double[] magnitude, Double[] peakCriteria)
   {

      boolean peakDetected = false;
      
      int peakIndexSpread = 4;
      {
      
      //Peak Detection Math
      int dominantFrequencyBandLowerBoundIndex = 0;
      int dominantFrequencyBandUpperBoundIndex = 0;
      int relevantFrequencyBandLowerBoundIndex = 0;
      int relevantFrequencyBandUpperBoundIndex = 0;

      double dominantBandLowerBound = peakCriteria[0];
      double dominantBandUpperBound = peakCriteria[1];
      double relevantBandLowerBound = peakCriteria[2];
      double relevantBandUpperBand = peakCriteria[3];
      double decibelsToTrip = peakCriteria[4];

      for (int index = 0; frequency[index] < dominantBandLowerBound; index++)
      {
         dominantFrequencyBandLowerBoundIndex = index;
      }

      for (int index = 0; frequency[index] < dominantBandUpperBound; index++)
      {
         dominantFrequencyBandUpperBoundIndex = index;
      }

      for (int index = 0; frequency[index] < relevantBandLowerBound; index++)
      {
         relevantFrequencyBandLowerBoundIndex = index;
      }

      for (int index = 0; frequency[index] < relevantBandUpperBand; index++)
      {
         relevantFrequencyBandUpperBoundIndex = index;
      }

      double dominantBandPeakMag = 0;
      double relevantBandAverageMag = 0;
      int dominantBandPeakIndex = 0;

      for (int index = dominantFrequencyBandLowerBoundIndex; index < dominantFrequencyBandUpperBoundIndex; index++)
      {
         if (magnitude[index] > dominantBandPeakMag)
         {
            dominantBandPeakMag = magnitude[index];
            dominantBandPeakIndex = index;
         }
      }
      
      dominantFrequencyBandLowerBoundIndex = dominantBandPeakIndex - peakIndexSpread;
      dominantFrequencyBandUpperBoundIndex = dominantBandPeakIndex + peakIndexSpread;
      
      if(dominantFrequencyBandLowerBoundIndex<=relevantFrequencyBandLowerBoundIndex){
         dominantFrequencyBandLowerBoundIndex=relevantFrequencyBandLowerBoundIndex+1;
         System.out.println("WARNING: SoundDetector dominant band range may need to be extended lower for the "+dominantFrequencyBandLowerBoundIndex+" Hz peak detection.");
      }
      
      for (int index = relevantFrequencyBandLowerBoundIndex; index < dominantFrequencyBandLowerBoundIndex; index++)
      {
         relevantBandAverageMag += magnitude[index];
      }
      
      if(relevantFrequencyBandUpperBoundIndex<=dominantFrequencyBandUpperBoundIndex){
         dominantFrequencyBandUpperBoundIndex=relevantFrequencyBandUpperBoundIndex-1;
         System.out.println("WARNING: SoundDetector dominant band range may need to be extended higher for the "+dominantFrequencyBandUpperBoundIndex+" Hz peak detection.");
      }

      for (int index = dominantFrequencyBandUpperBoundIndex; index < relevantFrequencyBandUpperBoundIndex; index++)
      {
         relevantBandAverageMag += magnitude[index];
      }

      relevantBandAverageMag = relevantBandAverageMag
            / ((relevantFrequencyBandUpperBoundIndex - dominantFrequencyBandUpperBoundIndex) + (dominantFrequencyBandLowerBoundIndex - relevantFrequencyBandLowerBoundIndex));

      if ((dominantBandPeakMag - relevantBandAverageMag) > decibelsToTrip)
      {
         peakDetected = true;
      }
      return peakDetected;

      }
   }

   public void stop()
   {
      thread = null;
   }

}