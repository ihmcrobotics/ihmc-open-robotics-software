package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.ArrayList;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredRateOfChangeOfMomentumCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumModuleDataObject;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumModuleSolution;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;


public class MomentumModuleSolutionComparer
{
   private static final boolean printComparison = true;
   
   private MomentumModuleDataObject momentumModuleDataObject;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final YoFrameVector desiredLinearMomentumRate, activeLinearMomentumRate, referenceLinearMomentumRate;
   private final YoFrameVector desiredAngularMomentumRate, activeAngularMomentumRate, referenceAngularMomentumRate;
   
   
   private SpatialForceVector activeCentroidalMomentumRate, referenceCentroidalMomentumRate;
   private ArrayList<ImmutablePair<String, Wrench>> activeExternalWrenches = new ArrayList<ImmutablePair<String,Wrench>>();
   private ArrayList<ImmutablePair<String, Wrench>> referenceExternalWrenches = new ArrayList<ImmutablePair<String,Wrench>>();
   
   
   public MomentumModuleSolutionComparer(ReferenceFrame centerOfMassFrame, YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      
      desiredLinearMomentumRate = new YoFrameVector("desiredLinearMomentumRate", centerOfMassFrame, registry);
      activeLinearMomentumRate = new YoFrameVector("activeLinearMomentumRate", centerOfMassFrame, registry);
      referenceLinearMomentumRate = new YoFrameVector("referenceLinearMomentumRate", centerOfMassFrame, registry);
      
      desiredAngularMomentumRate = new YoFrameVector("desiredAngularMomentumRate", centerOfMassFrame, registry);
      activeAngularMomentumRate = new YoFrameVector("activeAngularMomentumRate", centerOfMassFrame, registry);
      referenceAngularMomentumRate = new YoFrameVector("referenceAngularMomentumRate", centerOfMassFrame, registry);
   }
   
   public void setMomentumModuleDataObject(MomentumModuleDataObject momentumModuleDataObject)
   {
     this.momentumModuleDataObject = momentumModuleDataObject;  
   }
   
   public void setActiveSolution(String string, MomentumModuleSolution solution)
   {
      SpatialForceVector centroidalMomentumRateSolution = solution.getCentroidalMomentumRateSolution();
//      Map<RigidBody, Wrench> externalWrenchSolution = solution.getExternalWrenchSolution();
      
      activeCentroidalMomentumRate = new SpatialForceVector(centroidalMomentumRateSolution);
   }

   public void setReferenceSolution(String string, MomentumModuleSolution solution)
   {
      SpatialForceVector centroidalMomentumRateSolution = solution.getCentroidalMomentumRateSolution();
//      Map<RigidBody, Wrench> externalWrenchSolution = solution.getExternalWrenchSolution();
      
      referenceCentroidalMomentumRate = new SpatialForceVector(centroidalMomentumRateSolution);
   }

   public void displayComparison()
   {
      // Rate of change of momentum
      ArrayList<DesiredRateOfChangeOfMomentumCommand> desiredRateOfChangeOfMomentumCommands = momentumModuleDataObject.getDesiredRateOfChangeOfMomentumCommands();
      DesiredRateOfChangeOfMomentumCommand desiredRateOfChangeOfMomentumCommand = desiredRateOfChangeOfMomentumCommands.get(0);
      MomentumRateOfChangeData momentumRateOfChangeData = desiredRateOfChangeOfMomentumCommand.getMomentumRateOfChangeData();
      DenseMatrix64F momentumMultipliers = momentumRateOfChangeData.getMomentumMultipliers();
      DenseMatrix64F momentumSubspace = momentumRateOfChangeData.getMomentumSubspace();
      
      if (printComparison) System.out.println("momentumMultipliers = " + momentumMultipliers);
      if (printComparison) System.out.println("momentumSubspace = " + momentumSubspace);

      desiredLinearMomentumRate.set(momentumMultipliers.get(0), momentumMultipliers.get(1), momentumMultipliers.get(2));
      
      activeLinearMomentumRate.set(activeCentroidalMomentumRate.getLinearPartAsFrameVectorCopy());
      referenceLinearMomentumRate.set(referenceCentroidalMomentumRate.getLinearPartAsFrameVectorCopy());
      activeAngularMomentumRate.set(activeCentroidalMomentumRate.getAngularPartAsFrameVectorCopy());
      referenceAngularMomentumRate.set(referenceCentroidalMomentumRate.getAngularPartAsFrameVectorCopy());

      FrameVector angularMomentumRateDifference = new FrameVector(activeAngularMomentumRate.getFrameVectorCopy());
      angularMomentumRateDifference.sub(referenceAngularMomentumRate.getFrameVectorCopy());
      
      FrameVector linearMomentumRateDifference = activeLinearMomentumRate.getFrameVectorCopy();
      linearMomentumRateDifference.sub(referenceLinearMomentumRate.getFrameVectorCopy());
      
      if (printComparison) System.out.println("active CentroidalMomentumRate = " + activeCentroidalMomentumRate);
      if (printComparison) System.out.println("reference CentroidalMomentumRate = " + referenceCentroidalMomentumRate);
      
      if (printComparison) System.out.println("\nangularMomentumRateDifference = " + angularMomentumRateDifference);
      if (printComparison) System.out.println("linearMomentumRateDifference = " + linearMomentumRateDifference);

      
//      ArrayList<DesiredJointAccelerationCommand> desiredJointAccelerationCommands = momentumModuleDataObject.getDesiredJointAccelerationCommands(); 
//      for (DesiredJointAccelerationCommand desiredJointAccelerationCommand : desiredJointAccelerationCommands)
//      {
//         DenseMatrix64F desiredAcceleration = desiredJointAccelerationCommand.getDesiredAcceleration();
//         InverseDynamicsJoint joint = desiredJointAccelerationCommand.getJoint();
//         
//         DenseMatrix64F achievedAcceleration = new DenseMatrix64F(1, 1);
////         desiredJointAccelerationCommand.computeAchievedJointAcceleration(achievedAcceleration);
//         
//         if (printComparison) System.out.println(joint.getName() + ": " + desiredAcceleration + ", " + achievedAcceleration);
//      }
      
   }

   

}
