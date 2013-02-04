package us.ihmc.commonWalkingControlModules.controlModules;


public interface GroundReactionWrenchDistributor
{
   public abstract void solve(GroundReactionWrenchDistributorOutputData outputDataToPack, 
         GroundReactionWrenchDistributorInputData groundReactionWrenchDistributorInputData);
}