package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.trajectories.DoubleTrajectoryGenerator;


public class ZeroToOneParabolicVelocityTrajectoryGenerator implements DoubleTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final DoubleYoVariable trajectoryTime;
   private final DoubleYoVariable currentTime;

   private final DoubleYoVariable currentValue;
   private final DoubleYoVariable currentTimeDerivative;
   
   public ZeroToOneParabolicVelocityTrajectoryGenerator(String namePrefix, double trajectoryTime, YoVariableRegistry parentRegistry)
   {
      if(trajectoryTime <= 0.0)
         throw new RuntimeException("TrajectoryTime can not be less than or equal to 0.");
      
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.trajectoryTime = new DoubleYoVariable(namePrefix + "TrajectoryTime", registry);
      this.currentTime = new DoubleYoVariable(namePrefix + "Time", registry);
      this.currentValue = new DoubleYoVariable(namePrefix + "CurrentValue", registry);
      this.currentTimeDerivative = new DoubleYoVariable(namePrefix + "CurrentTimeDerivative", registry);
      
      this.trajectoryTime.set(trajectoryTime);
      
      parentRegistry.addChild(registry);
   }
   
   public void initialize()
   {
      currentTime.set(0.0);
      currentValue.set(0.0);
   }

   public void compute(double time)
   {
      this.currentTime.set(time);
      time = MathTools.clipToMinMax(time, 0.0, trajectoryTime.getDoubleValue());
      double finalTime = trajectoryTime.getDoubleValue();
      
      double output = -6.0 / MathTools.square(finalTime) * (MathTools.cube(time) / (3.0 * finalTime) - MathTools.square(time) / 2.0);
      currentValue.set(output);
      
      double timeDerivative = -6.0 * (-time + MathTools.square(time)/finalTime) / MathTools.square(finalTime);
      currentTimeDerivative.set(timeDerivative);
   }

   public boolean isDone()
   {
      return currentTime.getDoubleValue() >= trajectoryTime.getDoubleValue();
   }

   public double getValue()
   {
      return currentValue.getDoubleValue();
   }

   public double getVelocity()
   {
      return currentTimeDerivative.getDoubleValue();
   }

   public double getAcceleration()
   {
      return 0.0;
   }

   public void setTrajectoryTime(double trajectoryTime)
   {
      this.trajectoryTime.set(trajectoryTime);
   }
}
