package us.ihmc.commonWalkingControlModules.wrenchDistribution;


public interface GroundReactionWrenchDistributor
{
   public abstract void solve(GroundReactionWrenchDistributorOutputData outputDataToPack, 
         GroundReactionWrenchDistributorInputData groundReactionWrenchDistributorInputData);
}