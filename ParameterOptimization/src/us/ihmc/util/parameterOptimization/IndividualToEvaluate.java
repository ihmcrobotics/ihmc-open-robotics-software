package us.ihmc.util.parameterOptimization;

public abstract class IndividualToEvaluate
{   
   private boolean fitnessAlreadyComputed = false;
   private double cachedFitness = -1.0;
   private String name = "notNamed";
   
   public abstract IndividualToEvaluate createNewIndividual();
   
   public abstract ListOfParametersToOptimize getListOfParametersToOptimize();
   
   public abstract void startEvaluation();

   public abstract boolean isEvaluationDone();

   public abstract double computeFitness();
   
   public String getName()
   {
      return name;
   }
   
   public void setName(String name)
   {
      this.name = name;
   }
   
   public synchronized double getFitness()
   {
      if (fitnessAlreadyComputed)
      {
         // System.out.println("Returning cached value");
         return cachedFitness;
      }

      cachedFitness = this.computeFitness();
      fitnessAlreadyComputed = true;

      return cachedFitness;
   }
   
   protected synchronized void setFitness(double fitness)
   {
      fitnessAlreadyComputed = true;
      cachedFitness = fitness;
   }
}
