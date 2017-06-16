package us.ihmc.robotics.alphaToAlpha;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.trajectories.MinimumJerkTrajectory;

public class YoMiniJerkUpAndDownAlphaToAlpha implements AlphaToAlphaFunction
{
   private final YoDouble startOfRampUp;
   private final YoDouble endOfRamp;
   private final YoDouble startOfRampDown;
   private final YoDouble endOfRampDown;

   private final MinimumJerkTrajectory minimumJerkTrajectory;

   /*
      alpha < startOfRampUp => alphaPrime = 0
      startOfRampUp < alpha < endOfRamp => alphaPrime =  (alpha - startOfRampUp)/(endOfRamp - startOfRampUp)
      endOfRamp < alpha < startOfRampDown => alphaPrime = 1
      startOfRampDown < alpha < endOfRampDown => alphaPrime = 1 - (alpha - startOfRampDown)/(endOfRampDown - startOfRampDown)
      endOfRampDown < alpha => alphaPrime = 0

      you must set
      0.0 < startOfRampUp < endOfRamp < startOfRampDown < endOfRampDown < 1.0

      If this above condition is not met, then:
      alphaPrime will alway be ZERO!

    */
   public YoMiniJerkUpAndDownAlphaToAlpha(YoDouble startOfRampUp, YoDouble endOfRamp, YoDouble startOfRampDown, YoDouble endOfRampDown)
   {
      this.startOfRampUp = startOfRampUp;
      this.endOfRamp = endOfRamp;
      this.startOfRampDown = startOfRampDown;
      this.endOfRampDown = endOfRampDown;

      minimumJerkTrajectory = new MinimumJerkTrajectory();
   }

   @Override public double getAlphaPrime(double alpha)
   {
      if (!areVariablesInIncreasingOrderAndLessThanOne())
         return 0.0;

      if (alpha < startOfRampUp.getDoubleValue())
      {
         return 0.0;
      }
      else if(alpha < endOfRamp.getDoubleValue())
      {
         double rampUpFraction = alpha - startOfRampUp.getDoubleValue();
         minimumJerkTrajectory.setMoveParameters(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, endOfRamp.getDoubleValue() - startOfRampUp.getDoubleValue());
         minimumJerkTrajectory.computeTrajectory(rampUpFraction);

         return ( minimumJerkTrajectory.getPosition());
      }
      else if(alpha < startOfRampDown.getDoubleValue())
      {
         return 1.0;
      }
      else if(alpha < endOfRampDown.getDoubleValue())
      {
         double rampUpFraction = alpha - startOfRampDown.getDoubleValue();
         minimumJerkTrajectory.setMoveParameters(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, endOfRampDown.getDoubleValue() - startOfRampDown.getDoubleValue());
         minimumJerkTrajectory.computeTrajectory(rampUpFraction);

         return ( minimumJerkTrajectory.getPosition());
      }
      else
         return 0.0;

   }

   @Override public double getMaxAlpha()
   {
      return 1.0;
   }

   @Override public double getDerivativeAtAlpha(double alpha)
   {
      return Double.NaN;
   }

   @Override public double getSecondDerivativeAtAlpha(double alpha)
   {
      return Double.NaN;
   }

   private boolean areVariablesInIncreasingOrderAndLessThanOne()
   {
      if (startOfRampUp.getDoubleValue() <= 0.0)
         return false;
      if (endOfRamp.getDoubleValue() <= startOfRampUp.getDoubleValue())
         return false;
      if (startOfRampDown.getDoubleValue() <= endOfRamp.getDoubleValue())
         return false;
      if (endOfRampDown.getDoubleValue() <= startOfRampDown.getDoubleValue())
         return false;
      if (endOfRampDown.getDoubleValue() >= 1.0)
         return false;

      return true;
   }

   public static void main(String[] args)
   {
      YoVariableRegistry registry = new YoVariableRegistry("dummy");
      YoDouble startOfRampUp = new YoDouble("startOfRampUp", registry);
      YoDouble endOfRamp = new YoDouble("endOfRamp", registry);
      YoDouble startOfRampDown = new YoDouble("startOfRampDown", registry);
      YoDouble endOfRampDown = new YoDouble("endOfRampDown", registry);

      YoMiniJerkUpAndDownAlphaToAlpha yoMiniJerkUpAndDownAlphaToAlpha = new YoMiniJerkUpAndDownAlphaToAlpha(startOfRampUp, endOfRamp, startOfRampDown, endOfRampDown);

      startOfRampUp.set(0.1);
      endOfRamp.set(0.3);
      startOfRampDown.set(0.5);
      endOfRampDown.set(0.7);


      for(double alpha = 0.0; alpha <=1.0; alpha  = alpha + 0.01)
      {
         double alphaPrime = yoMiniJerkUpAndDownAlphaToAlpha.getAlphaPrime(alpha);
         System.out.println(alpha + ", " + alphaPrime);
      }


   }

}
