package us.ihmc.avatar.sensors.microphone;

import java.io.InputStream;

import javax.sound.sampled.AudioFormat;

import us.ihmc.commons.Conversions;
import us.ihmc.simulationconstructionset.gui.BodePlotConstructor;

public abstract class DrillDetectionAlgorithm
{
   private static final float sampleRate = 16000; //Hz
   private static final int sampleSizeInBits = 16;
   private static final int channels = 1;
   private static final boolean signed = true;
   private static final boolean bigEndian = false;
   private static final AudioFormat format = new AudioFormat(sampleRate, sampleSizeInBits, channels, signed, bigEndian);
   private static final int frameSizeInBytes = format.getFrameSize();
   private static final int bufferLengthInFrames = 16384 / 8;
   private static final int bufferLengthInBytes = bufferLengthInFrames * frameSizeInBytes;

   public float getSampleRate()
   {
      return format.getSampleRate();
   }

   public double[][] getBodeData(double[] time, double[] data)
   {
      double[][] freqMagPhase = BodePlotConstructor.computeFreqMagPhase(time, data);

      double[] frequency = freqMagPhase[0];
      double[] magnitude = Conversions.convertMagnitudeToDecibels(freqMagPhase[1]);
      double[] phase = Conversions.convertRadianToDegrees(freqMagPhase[2]);

      double[][] bodeData = new double[][] { frequency, magnitude, phase };
      return bodeData;
   }

   public DrillDetectionResult isDrillOn(InputStream inputStream)
   {
      if (format.getSampleSizeInBits() != 16)
      {
         System.out.println("Can't detect: bad sample size");
         return null;
      }

      try
      {
         byte[] data = new byte[bufferLengthInBytes];
         int numBytesRead = inputStream.read(data, 0, bufferLengthInBytes);
         return isDrillOn(data, numBytesRead);
      }
      catch (Exception ignored)
      {
         System.out.println("Failed to read from stream...");
         return null;
      }
   }

   public abstract DrillDetectionResult isDrillOn(byte[] audioBytes, int size);
   public abstract int getNumReturnedBands();
}