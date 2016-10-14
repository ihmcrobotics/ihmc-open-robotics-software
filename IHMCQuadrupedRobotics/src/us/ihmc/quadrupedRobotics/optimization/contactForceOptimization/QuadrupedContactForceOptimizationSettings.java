package us.ihmc.quadrupedRobotics.optimization.contactForceOptimization;

import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedContactForceOptimizationSettings
{
   public enum Solver
   {
      CONSTRAINED_QP,
      LINEAR
   }

   private final double MINIMUM_REGULARIZATION_WEIGHTS = 0.001;
   private final double[] comTorqueCommandWeights;
   private final double[] comForceCommandWeights;
   private final QuadrantDependentList<double[]> contactForceCommandWeights;
   private Solver solver;

   public QuadrupedContactForceOptimizationSettings()
   {
      comTorqueCommandWeights = new double[3];
      comForceCommandWeights = new double[3];
      contactForceCommandWeights = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactForceCommandWeights.set(robotQuadrant, new double[3]);
      }
      setDefaults();
   }

   public void setDefaults()
   {
      for (int i = 0; i < 3; i++)
      {
         comTorqueCommandWeights[i] = 1.0;
         comForceCommandWeights[i] = 1.0;
      }
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         for (int i = 0; i < 3; i++)
         {
            contactForceCommandWeights.get(robotQuadrant)[i] = MINIMUM_REGULARIZATION_WEIGHTS;
         }
      }
      setSolver(Solver.CONSTRAINED_QP);
   }

   public void setSolver(Solver solver)
   {
      this.solver = solver;
   }

   public void setComTorqueCommandWeights(double[] weights)
   {
      for (int i = 0; i < 3; i++)
      {
         comTorqueCommandWeights[i] = Math.max(weights[i], 0.0);
      }
   }

   public void setComTorqueCommandWeights(double weightX, double weightY, double weightZ)
   {
      comTorqueCommandWeights[0] = Math.max(weightX, 0.0);
      comTorqueCommandWeights[1] = Math.max(weightY, 0.0);
      comTorqueCommandWeights[2] = Math.max(weightZ, 0.0);
   }

   public void setComForceCommandWeights(double[] weights)
   {
      for (int i = 0; i < 3; i++)
      {
         comForceCommandWeights[i] = Math.max(weights[i], 0.0);
      }
   }

   public void setComForceCommandWeights(double weightX, double weightY, double weightZ)
   {
      comForceCommandWeights[0] = Math.max(weightX, 0.0);
      comForceCommandWeights[1] = Math.max(weightY, 0.0);
      comForceCommandWeights[2] = Math.max(weightZ, 0.0);
   }

   public void setContactForceCommandWeights(RobotQuadrant robotQuadrant, double[] weights)
   {
      for (int i = 0; i < 3; i++)
      {
         contactForceCommandWeights.get(robotQuadrant)[i] = Math.max(weights[i], MINIMUM_REGULARIZATION_WEIGHTS);
      }
   }

   public void setContactForceCommandWeights(RobotQuadrant robotQuadrant, double weightX, double weightY, double weightZ)
   {
      contactForceCommandWeights.get(robotQuadrant)[0] = Math.max(weightX, MINIMUM_REGULARIZATION_WEIGHTS);
      contactForceCommandWeights.get(robotQuadrant)[1] = Math.max(weightY, MINIMUM_REGULARIZATION_WEIGHTS);
      contactForceCommandWeights.get(robotQuadrant)[2] = Math.max(weightZ, MINIMUM_REGULARIZATION_WEIGHTS);
   }

   public /* const */ double[] getComTorqueCommandWeights()
   {
      return comTorqueCommandWeights;
   }

   public /* const */ double[] getComForceCommandWeights()
   {
      return comForceCommandWeights;
   }

   public /* const */ double[] getContactForceCommandWeights(RobotQuadrant robotQuadrant)
   {
      return contactForceCommandWeights.get(robotQuadrant);
   }

   public Solver getSolver()
   {
      return solver;
   }
}
