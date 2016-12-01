package us.ihmc.atlas.sensors;

import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.jtransforms.fft.FloatFFT_1D;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.tools.io.printing.PrintTools;

public class ForceSensorNoiseEstimator implements PacketConsumer<RobotConfigurationData>
{
   final private boolean DEBUG = false;
   final private static int NUM_SAMPLES = 256; // 128 is probably enough

   final private FloatFFT_1D fftCalculator = new FloatFFT_1D(NUM_SAMPLES); 
   private PacketCommunicator packetCommunicator;

   private Timer timer = new Timer();

   private class Channel
   {
      // data is passed between threads using a concurrent queue.
      public ConcurrentLinkedQueue<Float> dataFifo = new ConcurrentLinkedQueue<Float>();

      public float lastSample;
      
      public float[] circularBuffer = new float[NUM_SAMPLES];
      public int circularBufferIndex = 0;
      public float noiseIndicator;
   }

   private float[] orderedDataSamples = new float[NUM_SAMPLES];
   public  ArrayList<Channel> channels = new ArrayList<Channel>();

   public ForceSensorNoiseEstimator(PacketCommunicator sensorSuitePacketCommunicator)
   {
      this.packetCommunicator = sensorSuitePacketCommunicator;

      for (int i=0; i<Wrench.SIZE*2; i++)
      {
         channels.add( new Channel() );
      }

      timer.schedule( new NoiseCalculatorTimerTask(), 1000, 1000);
      
      PrintTools.info(this, "Constructed");
   }


   @Override
   public void receivedPacket(RobotConfigurationData packet)
   {
      // push all the data into the lock free fifo and notify the consumer
      for (int s=0; s<2; s++)
      {
         float [] force = packet.getMomentAndForceVectorForSensor(s);
         for (int f=0; f < Wrench.SIZE; f++)
         {
            channels.get(s* Wrench.SIZE + f).dataFifo.add( force[f] );
         }  
      }
   }


   class NoiseCalculatorTimerTask extends TimerTask
   {
      @Override
      public void run()
      { 
         if (DEBUG) System.out.println( "ForceSensorNoiseEstimator run ");
         
         Channel channel; 

         for( int c = 0; c < channels.size(); c++)
         {
            channel = channels.get(c);
            Float data =  channel.dataFifo.poll();
            while( data != null )
            {
               channel.circularBuffer [ channel.circularBufferIndex ] = data;

               data =  channel.dataFifo.poll();
               channel.circularBufferIndex++;
               channel.circularBufferIndex = channel.circularBufferIndex % NUM_SAMPLES;
            }
         }

         for( int c = 0; c < channels.size(); c++)
         {
            channel = channels.get(c);

            for (int i =0; i<NUM_SAMPLES; i++)
            {
               int indexCircular = (channel.circularBufferIndex + i) % NUM_SAMPLES;
               orderedDataSamples[i] = channel.circularBuffer[ indexCircular ];
            }

            // see the documentation of  FloatFFT_1D to see how the output data is packed.
            fftCalculator.realForward( orderedDataSamples );

            channel.noiseIndicator = 0;

            // note: this number has no physical meaning, it is just a rough amount of "noise".
            for (int s = 1; s < NUM_SAMPLES* 0.75/ 2.0; s++ )
            {
               channel.noiseIndicator += Math.abs( orderedDataSamples[s*2] ); // + Math.abs( orderedDataSamples[s*2+1] );
            }
            if (DEBUG) System.out.format( "%.2f\t", channel.noiseIndicator );
         }
         if (DEBUG)  System.out.println();
         
      }  
   }     


}
