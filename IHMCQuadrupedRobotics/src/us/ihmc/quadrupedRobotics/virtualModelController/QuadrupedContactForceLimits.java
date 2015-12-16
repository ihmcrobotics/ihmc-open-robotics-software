package us.ihmc.quadrupedRobotics.virtualModelController;

import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedContactForceLimits
{
   private final QuadrantDependentList<double[]> coefficientOfFriction;
   private final QuadrantDependentList<double[]> pressureLowerLimit;
   private final QuadrantDependentList<double[]> pressureUpperLimit;

   public QuadrupedContactForceLimits()
   {
      coefficientOfFriction = new QuadrantDependentList<double[]>();
      pressureLowerLimit = new QuadrantDependentList<double[]>();
      pressureUpperLimit = new QuadrantDependentList<double[]>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         coefficientOfFriction.set(robotQuadrant, new double[1]);
         pressureLowerLimit.set(robotQuadrant, new double[1]);
         pressureUpperLimit.set(robotQuadrant, new double[1]);
      }
      setDefaultLimits();
   }

   public void setDefaultLimits()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         setCoefficientOfFriction(robotQuadrant, 0.75);
         setPressureLowerLimit(robotQuadrant, 0.0);
         setPressureUpperLimit(robotQuadrant, Double.MAX_VALUE);
      }
   }

   public void setCoefficientOfFriction(RobotQuadrant robotQuadrant, double value)
   {
      coefficientOfFriction.get(robotQuadrant)[0] = value;
   }

   public void setPressureLowerLimit(RobotQuadrant robotQuadrant, double value)
   {
      pressureLowerLimit.get(robotQuadrant)[0] = value;
   }

   public void setPressureUpperLimit(RobotQuadrant robotQuadrant, double value)
   {
      pressureUpperLimit.get(robotQuadrant)[0] = value;
   }

   public double getCoefficientOfFriction(RobotQuadrant robotQuadrant)
   {
      return coefficientOfFriction.get(robotQuadrant)[0];
   }

   public double getPressureLowerLimit(RobotQuadrant robotQuadrant)
   {
      return pressureLowerLimit.get(robotQuadrant)[0];
   }

   public double getPressureUpperLimit(RobotQuadrant robotQuadrant)
   {
      return pressureUpperLimit.get(robotQuadrant)[0];
   }
}
