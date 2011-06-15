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
public class NonlinearObserverEncoderProcessor implements EncoderProcessor
{
   private final DoubleYoVariable rawPosition, discretePosition;
   private final DoubleYoVariable processedPosition, processedRate, error, alpha1, alpha2;

   private final double dt;
   private double unitDistancePerCount;
   
   public NonlinearObserverEncoderProcessor(String name, DoubleYoVariable rawPosition, double dt, YoVariableRegistry registry)
   {
      this.rawPosition = rawPosition;
      this.dt = dt;

      this.processedPosition = new DoubleYoVariable(name + "processedPosition", registry);
      this.discretePosition = new DoubleYoVariable(name + "discretePosition", registry);
      this.processedRate = new DoubleYoVariable(name + "processedRate", registry);
      this.error = new DoubleYoVariable(name + "error", registry);

      this.alpha1 = new DoubleYoVariable(name + "alpha1", registry);
      this.alpha2 = new DoubleYoVariable(name + "alpha2", registry);

      alpha1.set(0.03);
      alpha2.set(1.0);
   }
   
   public void update()
   {
      discretePosition.set((int) processedPosition.getDoubleValue());

      error.set(rawPosition.getDoubleValue() - discretePosition.getDoubleValue());

      processedRate.set(processedRate.getDoubleValue() + alpha2.getDoubleValue() * error.getDoubleValue());
      processedPosition.set(processedPosition.getDoubleValue() + processedRate.getDoubleValue() * dt + alpha1.getDoubleValue() * error.getDoubleValue());

   }

   public double getQ()
   {
      return this.processedPosition.getDoubleValue() * unitDistancePerCount;
   }

   public double getQd()
   {
      return this.processedRate.getDoubleValue() * unitDistancePerCount;
   }
   
   public void setUnitDistancePerCount(double unitDistancePerCount)
   {
      this.unitDistancePerCount = unitDistancePerCount;
   }
}
