package us.ihmc.commonWalkingControlModules.controlModules;


public interface GroundReactionWrenchDistributor
{
   public abstract void solve(GroundReactionWrenchDistributorInputData groundReactionWrenchDistributorInputData);
      
   public abstract GroundReactionWrenchDistributorOutputData getSolution();
   
   public abstract void getOutputData(GroundReactionWrenchDistributorOutputData outputData);
}