package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.ArrayList;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredRateOfChangeOfMomentumCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumModuleDataObject;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumModuleSolution;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialForceVector;
import us.ihmc.utilities.screwTheory.Wrench;

public class MomentumModuleSolutionComparer
{
   private MomentumModuleDataObject momentumModuleDataObject;

   private SpatialForceVector firstCentroidalMomentumRate, secondCentroidalMomentumRate;
   private ArrayList<Pair<String, Wrench>> firstExternalWrenches = new ArrayList<Pair<String,Wrench>>();
   private ArrayList<Pair<String, Wrench>> secondExternalWrenches = new ArrayList<Pair<String,Wrench>>();
   
   
   public MomentumModuleSolutionComparer()
   {
      
   }
   
   public void setMomentumModuleDataObject(MomentumModuleDataObject momentumModuleDataObject)
   {
     this.momentumModuleDataObject = momentumModuleDataObject;  
   }
   
   public void setFirstSolution(String string, MomentumModuleSolution firstSolution)
   {
      SpatialForceVector centroidalMomentumRateSolution = firstSolution.getCentroidalMomentumRateSolution();
      Map<RigidBody, Wrench> externalWrenchSolution = firstSolution.getExternalWrenchSolution();
      
      firstCentroidalMomentumRate = new SpatialForceVector(centroidalMomentumRateSolution);
   }

   public void setSecondSolution(String string, MomentumModuleSolution secondSolution)
   {
      SpatialForceVector centroidalMomentumRateSolution = secondSolution.getCentroidalMomentumRateSolution();
      Map<RigidBody, Wrench> externalWrenchSolution = secondSolution.getExternalWrenchSolution();
      
      secondCentroidalMomentumRate = new SpatialForceVector(centroidalMomentumRateSolution);
   }

   public void displayComparison()
   {
      ArrayList<DesiredRateOfChangeOfMomentumCommand> desiredRateOfChangeOfMomentumCommands = momentumModuleDataObject.getDesiredRateOfChangeOfMomentumCommands();
      DesiredRateOfChangeOfMomentumCommand desiredRateOfChangeOfMomentumCommand = desiredRateOfChangeOfMomentumCommands.get(0);
      MomentumRateOfChangeData momentumRateOfChangeData = desiredRateOfChangeOfMomentumCommand.getMomentumRateOfChangeData();
      DenseMatrix64F momentumMultipliers = momentumRateOfChangeData.getMomentumMultipliers();
      DenseMatrix64F momentumSubspace = momentumRateOfChangeData.getMomentumSubspace();
      
      System.out.println("momentumMultipliers = " + momentumMultipliers);
      System.out.println("momentumSubspace = " + momentumSubspace);

      FrameVector firstAngularMomentumRate = firstCentroidalMomentumRate.getAngularPartAsFrameVectorCopy();
      FrameVector secondAngularMomentumRate = secondCentroidalMomentumRate.getAngularPartAsFrameVectorCopy();
      
      FrameVector firstLinearMomentumRate = firstCentroidalMomentumRate.getLinearPartAsFrameVectorCopy();
      FrameVector secondLinearMomentumRate = secondCentroidalMomentumRate.getLinearPartAsFrameVectorCopy();

      FrameVector angularMomentumRateDifference = new FrameVector(firstAngularMomentumRate);
      angularMomentumRateDifference.sub(secondAngularMomentumRate);
      
      FrameVector linearMomentumRateDifference = new FrameVector(firstLinearMomentumRate);
      linearMomentumRateDifference.sub(secondLinearMomentumRate);
      
      System.out.println("First CentroidalMomentumRate = " + firstCentroidalMomentumRate);
      System.out.println("Second CentroidalMomentumRate = " + secondCentroidalMomentumRate);
      
      System.out.println("\nangularMomentumRateDifference = " + angularMomentumRateDifference);
      System.out.println("linearMomentumRateDifference = " + linearMomentumRateDifference);

   }

   

}
