package us.ihmc.sensorProcessing.drillDetection;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.util.concurrent.locks.ReentrantLock;

import org.apache.commons.math3.complex.Complex;
import org.apache.commons.math3.transform.DftNormalization;
import org.apache.commons.math3.transform.FastFourierTransformer;
import org.apache.commons.math3.transform.TransformType;
import org.jtransforms.fft.FloatFFT_1D;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.screwTheory.Wrench;

/*
 * Simple helper class to evaluate the amount of noise on force sensor.
 * This gives a good estimation of the state of the Drill (either On of off).
 * 
 * Usage:
 * 
 *  1) Continuously update using data from the force/torque sensors (method updateForceData ).
 *  2) Measure the amount of noise when you are sure that drill is OFF (method calculateNoiseIndicator ).
 *  3) make new measurements and compare the previous noiseIndicator with the current one. 
 *     You should see a considerably larger value.  
 */
public class DrillDetection
{
   final private static int numSamples = 128;
   final private Wrench wrenchToPack = new Wrench();

   final private double[][] data = new double[numSamples][6];
   final private double[][] tempData = new double[numSamples][6];

   final private float[] orderedSamples = new float[numSamples];

   final private FloatFFT_1D fftCalculator = new FloatFFT_1D(numSamples);

   int circularBufferIndex = -1;

   ReentrantLock lock = new ReentrantLock();

   public void updateForceData( double[] forceTorqueValue )
   {  
      lock.lock();

      boolean newValue = false;
      
      if( circularBufferIndex >= 0 )
      {
         // check if the value has been refreshed ( adding old value might affect FFT behavior )      
         for (int i=0; i<6; i++)
         {
            if( MathTools.epsilonEquals( data[circularBufferIndex][i], forceTorqueValue[i], 0.0001 ) == false )
            {
               newValue = true;
               break;
            }
         }
      }
      else{
         newValue = true;
      }

      if( newValue )
      {
         // circular buffer that overwrites old values.     
         circularBufferIndex = (circularBufferIndex + 1) % numSamples;         
         System.arraycopy(forceTorqueValue, 0, data[circularBufferIndex], 0, 6);
      }

      lock.unlock();
   }

   public double[] calculateNoiseIndicator()
   {
      double[] output = new double[6];
      
      lock.lock();
      int startingIndex = (circularBufferIndex + 1 ) % numSamples;

      for (int s=0; s<numSamples; s++ )
      {
         System.arraycopy( data[s] , 0, tempData[s], 0, 6);
      }
      lock.unlock();


      for (int i=0; i<6; i++ )
      {
         for (int s=0; s< numSamples; s++)
         {
            orderedSamples[s] = (float)tempData[ (s + startingIndex ) % numSamples  ][i];
         }

         fftCalculator.realForward( orderedSamples );

         output[i] = 0;

         // note: this number has no physical meaning, it is just a rough amount of "noise".
         for (int s = 1; s < numSamples* 0.75/ 2.0; s++ )
         {
            output[i] += Math.abs( orderedSamples[s*2] ); // + Math.abs( orderedSamples[s*2+1] );
         }
      }
      return output;
   }

   
   public static void main(String[] args) 
   {
      DrillDetection drillDetection = new DrillDetection();
      
      try{

         String[] fileNames = new String[]{"drillOff", "drillOn" };
 
         
         for (String filename: fileNames)
         {
            FileInputStream fstream = new FileInputStream("/home/unknownid/" + filename + ".txt");
            DataInputStream in = new DataInputStream(fstream);
            BufferedReader br = new BufferedReader(new InputStreamReader(in));

            String strLine;
            double[] sensorData = new double[6];

            float[]  sensorX_float  = new float[numSamples]; 
            double[] sensorX_double = new double[numSamples];

            int index = 0;
 
            while ((strLine = br.readLine()) != null)   {
               // Print the content on the console

               String[] elements = strLine.split(",");

               for(int i=0; i<6; i++)
               {
                  sensorData[i] = Double.parseDouble(elements[i]);
               }

               drillDetection.updateForceData( sensorData );

               int off = 500; 
               if(index >= off && index-off < numSamples)
               {
                  sensorX_float[index-off]  = (float) sensorData[4];
                  sensorX_double[index-off] =  sensorData[4];             
               }
               index++;
            }
            in.close();
            
            double[] sensorNoise = drillDetection.calculateNoiseIndicator();
            
            for(int i=0; i<6; i++)
            {
               System.out.print(sensorNoise[i] + " /  ");
            }
            System.out.println() ;
            

            FloatFFT_1D fftCalculator = new FloatFFT_1D( numSamples );
            fftCalculator.realForward( sensorX_float );

            FastFourierTransformer fft = new FastFourierTransformer(DftNormalization.STANDARD);
            Complex[] result =  fft.transform(sensorX_double, TransformType.FORWARD);

            //-------------------------------------------
            File fout = new File("/home/unknownid/" + filename + ".fft.txt");
            FileOutputStream fos = new FileOutputStream(fout);

            BufferedWriter bw = new BufferedWriter(new OutputStreamWriter(fos));

            if( true )
            {
               for (int i = 2; i < sensorX_float.length; i+=2 ) 
               {
                  bw.write( Math.abs(sensorX_float[i]) + " " + Math.abs( sensorX_float[i+1]));
                  bw.newLine();
               }
            }
            else{
               for (int i = 1; i < result.length/2; i++ ) 
               {
                  bw.write( Math.abs(result[i].getReal()) + " " + Math.abs(result[i].getImaginary()));
                  bw.newLine();
               }
            }
            bw.close();
         }
      }
      catch (Exception e){//Catch exception if any
         System.err.println("Error: " + e.getMessage());
      }
   }
    
}
