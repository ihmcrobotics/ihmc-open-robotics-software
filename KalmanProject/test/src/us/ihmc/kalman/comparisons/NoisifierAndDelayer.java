package us.ihmc.kalman.comparisons;

import java.util.Random;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.Robot;

public class NoisifierAndDelayer implements RobotController, ExampleFunctionController
{
   private final YoVariableRegistry registry = new YoVariableRegistry("Noisifier");
   private final String name;
   private final ExampleFunctionController function;

   private final YoDouble xWithNoise;
   private final YoDouble xDotWithNoise;
   private final YoDouble xDoubleDotWithNoise;

   private final Random random;

   private final double xStandardDeviation;
   private final double xDotStandardDeviation;
   private final double xDoubleDotStandardDeviation;
   
   private double lastMeasurement;
   private final YoInteger ticksPerMeasurement;
   private int tickFromLastMeasurement;

   private boolean isPositionMeasurementUpdated = false;

   public NoisifierAndDelayer(Robot robot, ExampleFunctionController function)
   {
      name = getClass().getSimpleName();
      this.function = function;
      
      ticksPerMeasurement = new YoInteger("ticksPerMeasurement", registry);
      ticksPerMeasurement.set(7);
      tickFromLastMeasurement = 0;

      xWithNoise = new YoDouble("xWithNoise", registry);
      xDotWithNoise = new YoDouble("xDotWithNoise", registry);
      xDoubleDotWithNoise = new YoDouble("xDoubleDotWithNoise", registry);

      random = new Random(234L);

      xStandardDeviation = 0.01;
      xDotStandardDeviation = 0.0;
      xDoubleDotStandardDeviation = 0.1;
   }

   public void initialize()
   {
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return getName();
   }

   public double getValue()
   {
      return xWithNoise.getDoubleValue();
   }

   public double getFirstDerivative()
   {
      return xDotWithNoise.getDoubleValue();
   }

   public double getSecondDerivative()
   {
      return xDoubleDotWithNoise.getDoubleValue();
   }

   public void doControl()
   {  
      
      tickFromLastMeasurement++;

      if (tickFromLastMeasurement == ticksPerMeasurement.getIntegerValue())
      {

         lastMeasurement = function.getValue() + random.nextGaussian() * xStandardDeviation;
         xWithNoise.set(lastMeasurement);
         isPositionMeasurementUpdated = true;
         tickFromLastMeasurement = 0;
         
      }
      else
         isPositionMeasurementUpdated = false;
      
      xDotWithNoise.set(function.getFirstDerivative() + random.nextGaussian() * xDotStandardDeviation);
      xDoubleDotWithNoise.set(function.getSecondDerivative() + random.nextGaussian() * xDoubleDotStandardDeviation);
   }

   public boolean isPositionMeasurementUpdated()
   {
      return isPositionMeasurementUpdated;
   }

}
