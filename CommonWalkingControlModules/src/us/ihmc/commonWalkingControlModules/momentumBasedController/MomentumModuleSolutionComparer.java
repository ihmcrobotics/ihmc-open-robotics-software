package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.ArrayList;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredRateOfChangeOfMomentumCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumModuleDataObject;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumModuleSolution;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialForceVector;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class MomentumModuleSolutionComparer
{
   private static final boolean printComparison = true;
   
   private MomentumModuleDataObject momentumModuleDataObject;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final YoFrameVector desiredLinearMomentumRate, optimizationLinearMomentumRate, oldLinearMomentumRate;
   private final YoFrameVector desiredAngularMomentumRate, optimizationAngularMomentumRate, oldAngularMomentumRate;
   
   
   private SpatialForceVector optimizationCentroidalMomentumRate, oldCentroidalMomentumRate;
   private ArrayList<Pair<String, Wrench>> optimizationExternalWrenches = new ArrayList<Pair<String,Wrench>>();
   private ArrayList<Pair<String, Wrench>> oldExternalWrenches = new ArrayList<Pair<String,Wrench>>();
   
   
   public MomentumModuleSolutionComparer(ReferenceFrame centerOfMassFrame, YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      
      desiredLinearMomentumRate = new YoFrameVector("desiredLinearMomentumRate", centerOfMassFrame, registry);
      optimizationLinearMomentumRate = new YoFrameVector("optimizationLinearMomentumRate", centerOfMassFrame, registry);
      oldLinearMomentumRate = new YoFrameVector("oldLinearMomentumRate", centerOfMassFrame, registry);
      
      desiredAngularMomentumRate = new YoFrameVector("desiredAngularMomentumRate", centerOfMassFrame, registry);
      optimizationAngularMomentumRate = new YoFrameVector("optimizationAngularMomentumRate", centerOfMassFrame, registry);
      oldAngularMomentumRate = new YoFrameVector("oldAngularMomentumRate", centerOfMassFrame, registry);
   }
   
   public void setMomentumModuleDataObject(MomentumModuleDataObject momentumModuleDataObject)
   {
     this.momentumModuleDataObject = momentumModuleDataObject;  
   }
   
   public void setOptimizationSolution(String string, MomentumModuleSolution optimizationSolution)
   {
      SpatialForceVector centroidalMomentumRateSolution = optimizationSolution.getCentroidalMomentumRateSolution();
      Map<RigidBody, Wrench> externalWrenchSolution = optimizationSolution.getExternalWrenchSolution();
      
      optimizationCentroidalMomentumRate = new SpatialForceVector(centroidalMomentumRateSolution);
   }

   public void setOldSolution(String string, MomentumModuleSolution oldSolution)
   {
      SpatialForceVector centroidalMomentumRateSolution = oldSolution.getCentroidalMomentumRateSolution();
      Map<RigidBody, Wrench> externalWrenchSolution = oldSolution.getExternalWrenchSolution();
      
      oldCentroidalMomentumRate = new SpatialForceVector(centroidalMomentumRateSolution);
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
      
      optimizationLinearMomentumRate.set(optimizationCentroidalMomentumRate.getLinearPartAsFrameVectorCopy());
      oldLinearMomentumRate.set(oldCentroidalMomentumRate.getLinearPartAsFrameVectorCopy());
      optimizationAngularMomentumRate.set(optimizationCentroidalMomentumRate.getAngularPartAsFrameVectorCopy());
      oldAngularMomentumRate.set(oldCentroidalMomentumRate.getAngularPartAsFrameVectorCopy());

      FrameVector angularMomentumRateDifference = new FrameVector(optimizationAngularMomentumRate.getFrameVectorCopy());
      angularMomentumRateDifference.sub(oldAngularMomentumRate.getFrameVectorCopy());
      
      FrameVector linearMomentumRateDifference = optimizationLinearMomentumRate.getFrameVectorCopy();
      linearMomentumRateDifference.sub(oldLinearMomentumRate.getFrameVectorCopy());
      
      if (printComparison) System.out.println("Optimization CentroidalMomentumRate = " + optimizationCentroidalMomentumRate);
      if (printComparison) System.out.println("Old CentroidalMomentumRate = " + oldCentroidalMomentumRate);
      
      if (printComparison) System.out.println("\nangularMomentumRateDifference = " + angularMomentumRateDifference);
      if (printComparison) System.out.println("linearMomentumRateDifference = " + linearMomentumRateDifference);

      
      ArrayList<DesiredJointAccelerationCommand> desiredJointAccelerationCommands = momentumModuleDataObject.getDesiredJointAccelerationCommands(); 
      for (DesiredJointAccelerationCommand desiredJointAccelerationCommand : desiredJointAccelerationCommands)
      {
         DenseMatrix64F desiredAcceleration = desiredJointAccelerationCommand.getDesiredAcceleration();
         InverseDynamicsJoint joint = desiredJointAccelerationCommand.getJoint();
         
         DenseMatrix64F achievedAcceleration = new DenseMatrix64F(1, 1);
         desiredJointAccelerationCommand.computeAchievedJointAcceleration(achievedAcceleration);
         
         if (printComparison) System.out.println(joint.getName() + ": " + desiredAcceleration + ", " + achievedAcceleration);
      }
      
   }

   

}
