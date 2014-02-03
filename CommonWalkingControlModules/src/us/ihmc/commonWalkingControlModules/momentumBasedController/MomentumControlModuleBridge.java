package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredPointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredRateOfChangeOfMomentumCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredSpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.ExternalWrenchCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumModuleDataObject;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumModuleSolution;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.gui.MomentumModuleGUI;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumControlModuleException;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumControlModuleSolverListener;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumControlModuleSolverVisualizer;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.OptimizationMomentumControlModule;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class MomentumControlModuleBridge implements MomentumControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry("MomentumControlModuleBridge");

   private static final boolean LISTEN_IN_ON_SOLVER = false;
   private static final boolean TRY_BOTH_AND_COMPARE = false;
   private static final boolean SHOW_MOMENTUM_MODULE_GUI =  false; //true;
   
   private final MomentumModuleSolutionComparer momentumModuleSolutionComparer;
   private final MomentumModuleGUI momentumModuleGUI;

   public enum MomentumControlModuleType 
   {
      OPTIMIZATION, OLD;
      public MomentumControlModuleType getOther()
      {
         if (this == OPTIMIZATION) return OLD;
         return OPTIMIZATION;
      }
   };

   private final EnumMap<MomentumControlModuleType, MomentumControlModule> momentumControlModules = new EnumMap<MomentumControlModuleType,
                                                                                                       MomentumControlModule>(MomentumControlModuleType.class);
   private final EnumYoVariable<MomentumControlModuleType> momentumControlModuleInUse =
      new EnumYoVariable<MomentumControlModuleType>("momentumControlModuleInUse", registry, MomentumControlModuleType.class);

   private final BooleanYoVariable swapMomentumControlModuleInUse = new BooleanYoVariable("swapMomentumControlModuleInUse", registry);
   private final BooleanYoVariable ignoreZDDot = new BooleanYoVariable("ignoreZDDot", registry);
   
   private MomentumControlModule activeMomentumControlModule, inactiveMomentumControlModule;

   private final MomentumModuleDataObject momentumModuleDataObject = new MomentumModuleDataObject();
   private final AllMomentumModuleListener allMomentumModuleListener;

   public MomentumControlModuleBridge(OptimizationMomentumControlModule optimizationMomentumControlModule, MomentumControlModule oldMomentumControlModule,
                                      ReferenceFrame centerOfMassFrame, YoVariableRegistry parentRegistry)
   {
      if (SHOW_MOMENTUM_MODULE_GUI)
      {
         momentumModuleGUI = new MomentumModuleGUI(registry);
         allMomentumModuleListener = new AllMomentumModuleListener(registry);
         
         optimizationMomentumControlModule.setPrimaryMotionConstraintListener(allMomentumModuleListener);
         optimizationMomentumControlModule.setSecondaryMotionConstraintListener(allMomentumModuleListener); 
         optimizationMomentumControlModule.setMomentumControlModuleSolverListener(allMomentumModuleListener);
      }
      else 
      {
         momentumModuleGUI = null;
         allMomentumModuleListener = null;
      }
         
      if (LISTEN_IN_ON_SOLVER) 
      {
         MomentumControlModuleSolverListener momentumControlModuleSolverListener = new MomentumControlModuleSolverVisualizer(registry);
         optimizationMomentumControlModule.setMomentumControlModuleSolverListener(momentumControlModuleSolverListener);
      }
      
      if (TRY_BOTH_AND_COMPARE) momentumModuleSolutionComparer = new MomentumModuleSolutionComparer(centerOfMassFrame, registry);
      else momentumModuleSolutionComparer = null;
      
      this.momentumControlModules.put(MomentumControlModuleType.OPTIMIZATION, optimizationMomentumControlModule);
      this.momentumControlModules.put(MomentumControlModuleType.OLD, oldMomentumControlModule);

      // By default use OptimizationMomentumControlModule, can be changed via setMomentumControlModuleToUse method
//      setMomentumControlModuleToUse(MomentumControlModuleType.OLD);
      setMomentumControlModuleToUse(MomentumControlModuleType.OPTIMIZATION);

      parentRegistry.addChild(registry);
   }

   public void swapMomentumControlModuleToUse()
   {
      setMomentumControlModuleToUse(momentumControlModuleInUse.getEnumValue().getOther());
   }
   
   public void setMomentumControlModuleToUse(MomentumControlModuleType momentumControlModuleToUse)
   {
      momentumControlModuleInUse.set(momentumControlModuleToUse);
      activeMomentumControlModule = momentumControlModules.get(momentumControlModuleToUse);

      if (TRY_BOTH_AND_COMPARE)
      {
         inactiveMomentumControlModule = momentumControlModules.get(momentumControlModuleToUse.getOther());
      }
      
      for (MomentumControlModule momentumControlModule : momentumControlModules.values())
      {
         if (momentumControlModule != null)
            momentumControlModule.initialize();
      }
   }

   public boolean isUsingOptimizationMomentumControlModule()
   {
      return (activeMomentumControlModule instanceof OptimizationMomentumControlModule);
   }

   public void initialize()
   {
      activeMomentumControlModule.initialize();

      if (SHOW_MOMENTUM_MODULE_GUI)
      {
         momentumModuleGUI.initialize();
      }
      
      if (TRY_BOTH_AND_COMPARE)
      {
         inactiveMomentumControlModule.initialize();
      }
   }

   public void reset()
   {
      if (swapMomentumControlModuleInUse.getBooleanValue())
      {
         swapMomentumControlModuleInUse.set(false);
         swapMomentumControlModuleToUse();
      }
      
      activeMomentumControlModule.reset();
      momentumModuleDataObject.reset();
      
      if (SHOW_MOMENTUM_MODULE_GUI)
      {
         allMomentumModuleListener.reset();
         momentumModuleGUI.reset();
      }
      
      if (TRY_BOTH_AND_COMPARE)
      {
         inactiveMomentumControlModule.reset();
      }
   }

   public void resetGroundReactionWrenchFilter()
   {
      activeMomentumControlModule.resetGroundReactionWrenchFilter();
      
      if (TRY_BOTH_AND_COMPARE)
      {
         inactiveMomentumControlModule.resetGroundReactionWrenchFilter();
      }
   }

   public void setDesiredRateOfChangeOfMomentum(DesiredRateOfChangeOfMomentumCommand desiredRateOfChangeOfMomentumCommand)
   {
      MomentumRateOfChangeData momentumRateOfChangeData = desiredRateOfChangeOfMomentumCommand.getMomentumRateOfChangeData();
      DenseMatrix64F momentumMultipliers = momentumRateOfChangeData.getMomentumMultipliers();
      DenseMatrix64F momentumSubspace = momentumRateOfChangeData.getMomentumSubspace();
      
//      System.out.println("momentumMultipliers = " + momentumMultipliers);
//      System.out.println("momentumSubspace = " + momentumSubspace);
      
      if (ignoreZDDot.getBooleanValue())
      {
//         DenseMatrix64F newMomentumMultipliers = new DenseMatrix64F(momentumMultipliers);
//         DenseMatrix64F newMomentumSubspace = new DenseMatrix64F(momentumSubspace);

         DenseMatrix64F newMomentumMultipliers = new DenseMatrix64F(2, 1);
         newMomentumMultipliers.set(0, 0, momentumMultipliers.get(0, 0));
         newMomentumMultipliers.set(1, 0, momentumMultipliers.get(1, 0));
         
         DenseMatrix64F newMomentumSubspace = new DenseMatrix64F(6, 2);
         newMomentumSubspace.set(3, 0, 1.0);
         newMomentumSubspace.set(4, 1, 1.0);
         
//         System.out.println("newMomentumMultipliers = " + newMomentumMultipliers);
//         System.out.println("newMomentumSubspace = " + newMomentumSubspace);
         
         momentumRateOfChangeData.setMomentumMultipliers(newMomentumMultipliers);
         momentumRateOfChangeData.setMomentumSubspace(newMomentumSubspace);
         
         desiredRateOfChangeOfMomentumCommand.setMomentumRateOfChangeData(momentumRateOfChangeData);
      }
      
      momentumModuleDataObject.setDesiredRateOfChangeOfMomentum(desiredRateOfChangeOfMomentumCommand);
      if (allMomentumModuleListener != null) allMomentumModuleListener.desiredRateOfChangeOfMomentumWasSet(desiredRateOfChangeOfMomentumCommand);
   }

   public void setDesiredJointAcceleration(DesiredJointAccelerationCommand desiredJointAccelerationCommand)
   {
      momentumModuleDataObject.setDesiredJointAcceleration(desiredJointAccelerationCommand);
      if (allMomentumModuleListener != null) allMomentumModuleListener.desiredJointAccelerationWasSet(desiredJointAccelerationCommand);
   }

   public void setDesiredSpatialAcceleration(DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand)
   {
      momentumModuleDataObject.setDesiredSpatialAcceleration(desiredSpatialAccelerationCommand);
      if (allMomentumModuleListener != null) allMomentumModuleListener.desiredSpatialAccelerationWasSet(desiredSpatialAccelerationCommand);
   }
   
   public void setDesiredPointAcceleration(DesiredPointAccelerationCommand desiredPointAccelerationCommand)
   {
      momentumModuleDataObject.setDesiredPointAcceleration(desiredPointAccelerationCommand);
      if (allMomentumModuleListener != null) allMomentumModuleListener.desiredPointAccelerationWasSet(desiredPointAccelerationCommand);
   }

   public void setExternalWrenchToCompensateFor(RigidBody rigidBody, Wrench wrench)
   {
      momentumModuleDataObject.setExternalWrenchToCompensateFor(rigidBody, wrench);
   }

   public MomentumModuleSolution compute(Map<ContactablePlaneBody, ? extends PlaneContactState> contactStates, RobotSide upcomingSupportSide)
           throws MomentumControlModuleException
   {
      MomentumModuleSolution inactiveSolution = null;
      
      if (TRY_BOTH_AND_COMPARE)
      {
         setMomentumModuleDataObject(inactiveMomentumControlModule, momentumModuleDataObject);
         inactiveSolution = inactiveMomentumControlModule.compute(contactStates, upcomingSupportSide);
      
         momentumModuleSolutionComparer.setMomentumModuleDataObject(momentumModuleDataObject);
         if (this.isUsingOptimizationMomentumControlModule()) momentumModuleSolutionComparer.setOldSolution("Old Solution", inactiveSolution);
         else momentumModuleSolutionComparer.setOptimizationSolution("Optimization Solution", inactiveSolution);
      }
      
      setMomentumModuleDataObject(activeMomentumControlModule, momentumModuleDataObject);
      MomentumModuleSolution activeSolution = activeMomentumControlModule.compute(contactStates, upcomingSupportSide);  
  
      if (allMomentumModuleListener != null)
      {
         allMomentumModuleListener.momentumModuleSolutionWasComputed(activeSolution);
      }
      
      if (SHOW_MOMENTUM_MODULE_GUI)
      {
         momentumModuleGUI.update(allMomentumModuleListener);
//         momentumModuleGUI.setDesiredsAndSolution(momentumModuleDataObject, activeSolution);
      }
      
      if (TRY_BOTH_AND_COMPARE)
      {
         if (this.isUsingOptimizationMomentumControlModule()) momentumModuleSolutionComparer.setOptimizationSolution("Optimization Solution", activeSolution);
         else momentumModuleSolutionComparer.setOldSolution("Old Solution", activeSolution);
         
         momentumModuleSolutionComparer.displayComparison();
      }

      
      return activeSolution;
   }

   private static void setMomentumModuleDataObject(MomentumControlModule momentumControlModule, MomentumModuleDataObject momentumModuleDataObject)
   {
      ArrayList<DesiredRateOfChangeOfMomentumCommand> desiredRateOfChangeOfMomentumCommands =
         momentumModuleDataObject.getDesiredRateOfChangeOfMomentumCommands();
      for (DesiredRateOfChangeOfMomentumCommand desiredRateOfChangeOfMomentumCommand : desiredRateOfChangeOfMomentumCommands)
      {
         setDesiredRateOfChangeOfMomentum(momentumControlModule, desiredRateOfChangeOfMomentumCommand);
      }

      ArrayList<DesiredJointAccelerationCommand> desiredJointAccelerationCommands = momentumModuleDataObject.getDesiredJointAccelerationCommands();
      for (DesiredJointAccelerationCommand desiredJointAccelerationCommand : desiredJointAccelerationCommands)
      {
         setDesiredJointAcceleration(momentumControlModule, desiredJointAccelerationCommand);
      }

      ArrayList<DesiredSpatialAccelerationCommand> desiredSpatialAccelerationCommands = momentumModuleDataObject.getDesiredSpatialAccelerationCommands();
      for (DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand : desiredSpatialAccelerationCommands)
      {
         setDesiredSpatialAcceleration(momentumControlModule, desiredSpatialAccelerationCommand);
      }

      ArrayList<DesiredPointAccelerationCommand> desiredPointAccelerationCommands = momentumModuleDataObject.getDesiredPointAccelerationCommands();
      for (DesiredPointAccelerationCommand desiredPointAccelerationCommand : desiredPointAccelerationCommands)
      {
         setDesiredPointAcceleration(momentumControlModule, desiredPointAccelerationCommand);
      }

      ArrayList<ExternalWrenchCommand> externalWrenchCommands = momentumModuleDataObject.getExternalWrenchCommands();
      for (ExternalWrenchCommand externalWrenchCommand : externalWrenchCommands)
      {
         setExternalWrenchToCompensateFor(momentumControlModule, externalWrenchCommand);
      }
   }


   private static void setDesiredRateOfChangeOfMomentum(MomentumControlModule momentumControlModule,
           DesiredRateOfChangeOfMomentumCommand desiredRateOfChangeOfMomentumCommand)
   {
      momentumControlModule.setDesiredRateOfChangeOfMomentum(desiredRateOfChangeOfMomentumCommand);
   }

   private static void setDesiredJointAcceleration(MomentumControlModule momentumControlModule, DesiredJointAccelerationCommand desiredJointAccelerationCommand)
   {
      momentumControlModule.setDesiredJointAcceleration(desiredJointAccelerationCommand);
   }

   private static void setDesiredSpatialAcceleration(MomentumControlModule momentumControlModule,
           DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand)
   {
      momentumControlModule.setDesiredSpatialAcceleration(desiredSpatialAccelerationCommand);
   }


   private static void setDesiredPointAcceleration(MomentumControlModule momentumControlModule, DesiredPointAccelerationCommand desiredPointAccelerationCommand)
   {
      momentumControlModule.setDesiredPointAcceleration(desiredPointAccelerationCommand);
   }

   private static void setExternalWrenchToCompensateFor(MomentumControlModule momentumControlModule, ExternalWrenchCommand externalWrenchCommand)
   {
      momentumControlModule.setExternalWrenchToCompensateFor(externalWrenchCommand.getRigidBody(), externalWrenchCommand.getWrench());
   }

}
