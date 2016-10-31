package us.ihmc.kalman.comparisons;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.Robot;

public class SinosoidalExampleFunction  implements RobotController, ExampleFunctionController
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SinosoidalExampleFunction");
   private final String name;
   private final Robot robot;
   
   private final DoubleYoVariable xReal;
   private final DoubleYoVariable xDotReal;
   private final DoubleYoVariable xDoubleDotReal;
   
   private final double amplitude;
   private final double frequency;
   private final double phase;
   
   protected final DoubleYoVariable time = new DoubleYoVariable("time", registry);
   protected final DoubleYoVariable dt = new DoubleYoVariable("dt", registry);
   private double tLastTick;
   
   public SinosoidalExampleFunction(Robot robot, double amplitude, double frequency, double phase)
   {
      name = getClass().getSimpleName();
      this.robot = robot;
      this.amplitude = amplitude;
      this.frequency = frequency;
      this.phase = phase;
      
      tLastTick = robot.getTime();
      
      xReal = new DoubleYoVariable("xReal", registry);
      xDotReal = new DoubleYoVariable("xDotReal", registry);
      xDoubleDotReal = new DoubleYoVariable("xDoubleDotReal", registry);
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
