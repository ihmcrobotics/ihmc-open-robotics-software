package us.ihmc.kalman.comparisons;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.Robot;

public class SinosoidalExampleFunction  implements RobotController, ExampleFunctionController
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SinosoidalExampleFunction");
   private final String name;
   private final Robot robot;
   
   private final YoDouble xReal;
   private final YoDouble xDotReal;
   private final YoDouble xDoubleDotReal;
   
   private final double amplitude;
   private final double frequency;
   private final double phase;
   
   protected final YoDouble time = new YoDouble("time", registry);
   protected final YoDouble dt = new YoDouble("dt", registry);
   private double tLastTick;
   
   public SinosoidalExampleFunction(Robot robot, double amplitude, double frequency, double phase)
   {
      name = getClass().getSimpleName();
      this.robot = robot;
      this.amplitude = amplitude;
      this.frequency = frequency;
      this.phase = phase;
      
      tLastTick = robot.getTime();
      
      xReal = new YoDouble("xReal", registry);
      xDotReal = new YoDouble("xDotReal", registry);
      xDoubleDotReal = new YoDouble("xDoubleDotReal", registry);
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


   public void doControl()
   {
      time.set(robot.getTime());
      dt.set(time.getDoubleValue() - tLastTick);
      tLastTick = time.getDoubleValue();
      
      xReal.set(value());
      xDotReal.set(firstDerivative());
      xDoubleDotReal.set(secondDerivative());
   }

   private double value()
   {
      return amplitude * Math.sin(frequency * time.getDoubleValue() + phase);
   }
   
   private double firstDerivative()
   {
      return amplitude * frequency * Math.cos(frequency * time.getDoubleValue() + phase);
   }
   
   private double secondDerivative()
   {
      return - frequency * frequency * value();
   }
   
   /* (non-Javadoc)
    * @see us.ihmc.kalman.ExampleFunctionController#getValue()
    */
   public double getValue()
   {
      return xReal.getDoubleValue();
   }
   
   /* (non-Javadoc)
    * @see us.ihmc.kalman.ExampleFunctionController#getFirstDerivative()
    */
   public double getFirstDerivative()
   {
      return xDotReal.getDoubleValue();
   }
   
   /* (non-Javadoc)
    * @see us.ihmc.kalman.ExampleFunctionController#getSecondDerivative()
    */
   public double getSecondDerivative()
   {
      return xDotReal.getDoubleValue();
   }


   public double getDT()
   {
      return dt.getDoubleValue();
   }


   public boolean isPositionMeasurementUpdated()
   {
      return true;
   }
}
