package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

public class CentroidalZAxisOptimizationControlModule
{
   // Planner parameters
   private final double mass;
   private final double Izz;
   private final double deltaTMin;

   // Runtime variables for optimization
   private int numberOfNodes;
   
   public CentroidalZAxisOptimizationControlModule(int maxNumberOfNodes, double robotMass, double Izz, double deltaTMin)
   {
      this.mass = robotMass;
      this.Izz = Izz;
      this.deltaTMin = deltaTMin;
   }
}
