package us.ihmc.sensorProcessing.encoder.processors;


import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class AnotherEncoderProcessor implements EncoderProcessor
{
   private final DoubleYoVariable rawPosition, time;
   private final DoubleYoVariable processedPosition, processedRate;

   private double unitDistancePerCount;
   
   private double previousTime;
   private double previousPosition;
   private boolean previousTickUp;

   public AnotherEncoderProcessor(String name, DoubleYoVariable rawPosition, DoubleYoVariable time, YoVariableRegistry registry)
   {
      this.rawPosition = rawPosition;
      this.time = time;

      this.processedPosition = new DoubleYoVariable(name + "processedPosition", registry);
      this.processedRate = new DoubleYoVariable(name + "processedRate", registry);
   }

   public double getQ()
   {
      return this.processedPosition.getDoubleValue();
   }

   public double getQd()
   {
      return this.processedRate.getDoubleValue();
   }


   public void update()
   {
//    this.processedPosition.val = rawPosition.val;
//    this.processedRate.val =
      double time = this.time.getDoubleValue();
      double position = this.rawPosition.getDoubleValue();

      boolean thisTickUp;

      if (position == previousPosition)
      {
         this.processedRate.set((position - previousPosition) / (time - previousTime));

         return;
      }

      if (position > previousPosition)
      {
         thisTickUp = true;
      }
      else
      {
         thisTickUp = false;
      }

      if (thisTickUp == previousTickUp)
      {
         this.processedPosition.set(rawPosition.getDoubleValue());
         this.processedRate.set((position - previousPosition) / (time - previousTime));

         previousPosition = position;
         previousTime = time;
         previousTickUp = thisTickUp;

         return;
      }

      else
      {
         this.processedRate.set(0.0);

         previousPosition = position;
         previousTime = time;
         previousTickUp = thisTickUp;
      }
   }
   
   public void setUnitDistancePerCount(double unitDistancePerCount)
   {
      this.unitDistancePerCount = unitDistancePerCount;
   }
}
