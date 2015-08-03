package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.robotics.MathTools;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.trajectories.DoubleTrajectoryGenerator;


public class LimitedCenterOfMassHeightTrajectoryGenerator implements DoubleTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final DoubleYoVariable desiredCenterOfMassHeight;
   private final DoubleYoVariable desiredCenterOfMassHeightFinal;
   private final DoubleYoVariable maxCenterOfMassHeight;
   private final DoubleYoVariable minCenterOfMassHeight;
   
   private final DoubleYoVariable maxCoMHeightDot;
   private final DoubleYoVariable desiredCenterOfMassHeightVelocity;
   private final double controlDT;

   public LimitedCenterOfMassHeightTrajectoryGenerator(double controlDT, double initialDesiredHeight, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(getClass().getSimpleName());
      desiredCenterOfMassHeight = new DoubleYoVariable("desiredCenterOfMassHeight", registry); 
      desiredCenterOfMassHeightFinal = new DoubleYoVariable("desiredCenterOfMassHeightFinal", registry); 
      maxCenterOfMassHeight = new DoubleYoVariable("maxCenterOfMassHeight", registry); 
      minCenterOfMassHeight = new DoubleYoVariable("minCenterOfMassHeightFinal", registry); 
      
      parentRegistry.addChild(registry);

      desiredCenterOfMassHeight.set(initialDesiredHeight);
      
      maxCoMHeightDot = new DoubleYoVariable("maxCoMHeightDot", registry);
      desiredCenterOfMassHeightVelocity = new DoubleYoVariable("desiredCenterOfMassHeightVelocity", registry);
      this.controlDT = controlDT;
      
      desiredCenterOfMassHeightFinal.set(1.15);
      maxCoMHeightDot.set(0.1);
      
   }

   public void setFinalDesiredHeight(double height)
   {
      this.desiredCenterOfMassHeightFinal.set(height);
   }
   
   public void setMaximumDesiredHeight(double maxHeight)
   {
      maxCenterOfMassHeight.set(maxHeight);
   }
   
   public void setMinimumDesiredHeight(double minHeight)
   {
      minCenterOfMassHeight.set(minHeight);
   }

   public void initialize()
   {
      // empty
   }

   public void compute(double time)
   {
      double finalHeight = MathTools.clipToMinMax(desiredCenterOfMassHeightFinal.getDoubleValue(), minCenterOfMassHeight.getDoubleValue(), maxCenterOfMassHeight.getDoubleValue());
      double error = finalHeight - desiredCenterOfMassHeight.getDoubleValue();
      double maximumChangePerTick = maxCoMHeightDot.getDoubleValue() * controlDT;

      double deltaHeading = MathTools.clipToMinMax(error, -maximumChangePerTick, maximumChangePerTick);
      desiredCenterOfMassHeight.set(desiredCenterOfMassHeight.getDoubleValue() + deltaHeading);
      double velocity = deltaHeading / controlDT;
      desiredCenterOfMassHeightVelocity.set(velocity);
   }

   public boolean isDone()
   {
      return true;
   }

   public double getValue()
   {
      return desiredCenterOfMassHeight.getDoubleValue();
   }

   public double getVelocity()
   {
      return desiredCenterOfMassHeightVelocity.getDoubleValue();
   }

   public double getAcceleration()
   {
      return 0.0;
   }
}
