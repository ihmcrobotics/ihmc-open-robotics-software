package us.ihmc.aware.vmc;

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
   private double contactForceRegularizationWeights;
   private Solver solver;

   public QuadrupedContactForceOptimizationSettings()
   {
      comTorqueCommandWeights = new double[3];
      comForceCommandWeights = new double[3];
      setDefaultSettings();
   }

   public void setDefaultSettings()
   {
      for (int i = 0; i < 3; i++)
      {
         comTorqueCommandWeights[i] = 1.0;
         comForceCommandWeights[i] = 1.0;
      }
      contactForceRegularizationWeights = MINIMUM_REGULARIZATION_WEIGHTS;
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

   public void setComForceCommandWeights(double[] weights)
   {
      for (int i = 0; i < 3; i++)
      {
         comForceCommandWeights[i] = Math.max(weights[i], 0.0);
      }
   }

   public void setContactForceRegularizationWeights(double weights)
   {
      contactForceRegularizationWeights = Math.max(weights, MINIMUM_REGULARIZATION_WEIGHTS);
   }

   public /* const */ double[] getComTorqueCommandWeights()
   {
      return comTorqueCommandWeights;
   }

   public /* const */ double[] getComForceCommandWeights()
   {
      return comForceCommandWeights;
   }

   public double getContactForceRegularizationWeights()
   {
      return contactForceRegularizationWeights;
   }

   public Solver getSolver()
   {
      return solver;
   }
}
