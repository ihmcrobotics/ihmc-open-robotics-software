package us.ihmc.valkyrie.encoder.comparison;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

import us.ihmc.sensorProcessing.encoder.processors.EncoderUnwrapper;


public class RealLifeEncoderTrajectory
{

   private int[] ticks=null;
   private double[] pos=null;
   private double[] vel=null;
   private int[] heartBeat=null;
   private int cursor=0;
   private double freq = 500;
   private double dt = 1.0/freq;
   private int tickPerRev = 8192;
   private int sign = 1;
   private EncoderUnwrapper unwrapper;
   
   public RealLifeEncoderTrajectory(String logFileName, String jointName) throws IOException
   {
      BufferedReader reader  = new BufferedReader(new FileReader(logFileName));
      String line;
      while((line = reader.readLine()) != null)
      {
         String[] cols = line.split("=");
         System.out.println("JointName"+ cols[0]);
         if(cols[0].contains(jointName))
         {
            String[] vals = cols[1].split(" ");
            if(cols[0].contains("DigAPS_Raw"))
            {
               ticks = new int[vals.length-3]; //cut head and tail;
               int count=0;
               for(int i=2;i<vals.length-1;i++)
               {
                  ticks[count++]=(int)Double.parseDouble(vals[i]);
               }
            }
            else if(cols[0].contains("Angle_Rad"))
            {
               pos = new double[vals.length-3];
               int count=0;
               for(int i=2;i<vals.length-1;i++)
               {
                  pos[count++]=Double.parseDouble(vals[i]);
               }
            }
            else if(cols[0].contains("Vel_Radps"))
            {
               vel = new double[vals.length-3];
               int count=0;
               for(int i=2;i<vals.length-1;i++)
               {
                  vel[count++]=Double.parseDouble(vals[i]);
               }
            }
            else if(cols[0].contains("Proc_HeartBeat"))
            {
               heartBeat = new int[vals.length-3];
               int count=0;
               for(int i=2;i<vals.length-1;i++)
               {
                  heartBeat[count++]=(int)Double.parseDouble(vals[i]);
               }
            }
         }

      }
      reader.close();
      
      if(pos==null || ticks==null || vel==null)
      {
         throw new RuntimeException("JointName not found");
      }
      else
      {
         sign = -(int)Math.signum( ((vel[vel.length-1]-vel[0])*(ticks[ticks.length-1]-ticks[0])));
      }

      unwrapper = new EncoderUnwrapper(tickPerRev);
   }
  
   public double getDt()
   {
      return dt;
   }
   
   
  
   public int getNumTicks()
   {
      return ticks.length;
   }
  
   public int getHeartBeat()
   {
      return heartBeat[cursor];
   }
   public double getTime()
   {
      return cursor*dt;
   }
   
   
   public void nextTimestep()
   {
      cursor++;
   }

   public int getEncoderTick()
   {
      return ticks[cursor];
   }
   
   public int getUnwrappedEncoderTicks()
   {
      return unwrapper.update(getEncoderTick());
   }
   
   public double getPosition()
   {
      // TODO Auto-generated method stub
      return sign*pos[cursor];
   }

   public double getVelocity()
   {
      // TODO Auto-generated method stub
      return sign*vel[cursor];
   }

   public double tickPerRev()
   {
      return tickPerRev;
   }

}
