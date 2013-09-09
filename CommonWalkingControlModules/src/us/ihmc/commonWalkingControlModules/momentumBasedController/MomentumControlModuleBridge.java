package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactableCylinderBody;
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
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumControlModuleSolverListener;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumControlModuleSolverVisualizer;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.OptimizationMomentumControlModule;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.CylindricalContactState;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.exeptions.NoConvergenceException;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
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
   private final MomentumModuleSolutionComparer momentumModuleSolutionComparer;

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
   
   private MomentumControlModule activeMomentumControlModule, inactiveMomentumControlModule;

   private final MomentumModuleDataObject momentumModuleDataObject = new MomentumModuleDataObject();

   public MomentumControlModuleBridge(OptimizationMomentumControlModule optimizationMomentumControlModule, MomentumControlModule oldMomentumControlModule,
                                      ReferenceFrame centerOfMassFrame, YoVariableRegistry parentRegistry)
   {
      if (LISTEN_IN_ON_SOLVER) 
      {
         MomentumControlModuleSolverListener momentumControlModuleSolverListener = new MomentumControlModuleSolverVisualizer(registry);
         optimizationMomentumControlModule.setMomentumControlModuleSolverListener(momentumControlModuleSolverListener);
      }
      
      if (TRY_BOTH_AND_COMPARE) momentumModuleSolutionComparer = new MomentumModuleSolutionComparer(centerOfMassFrame, registry);
      else momentumModuleSolutionComparer = null;
      
      this.momentumControlModules.put(MomentumControlModuleType.OPTIMIZATION, optimizationMomentumControlModule);
      this.momentumControlModules.put(MomentumControlModuleType.OLD, oldMomentumControlModule);

      // By default use OldMomentumControlModule, can be changed via setMomentumControlModuleToUse method
      setMomentumControlModuleToUse(MomentumControlModuleType.OLD);

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

   public void setDesiredRateOfChangeOfMomentum(MomentumRateOfChangeData momentumRateOfChangeData)
   {
      momentumModuleDataObject.setDesiredRateOfChangeOfMomentum(momentumRateOfChangeData);
   }

   public void setDesiredJointAcceleration(InverseDynamicsJoint joint, DenseMatrix64F jointAcceleration)
   {
      momentumModuleDataObject.setDesiredJointAcceleration(joint, jointAcceleration);
   }

   public void setDesiredJointAcceleration(InverseDynamicsJoint joint, DenseMatrix64F jointAcceleration, double weight)
   {
      momentumModuleDataObject.setDesiredJointAcceleration(joint, jointAcceleration, weight);
   }

   public void setDesiredSpatialAcceleration(GeometricJacobian jacobian, TaskspaceConstraintData taskspaceConstraintData)
   {
      momentumModuleDataObject.setDesiredSpatialAcceleration(jacobian, taskspaceConstraintData);
   }

   public void setDesiredSpatialAcceleration(GeometricJacobian jacobian, TaskspaceConstraintData taskspaceConstraintData, double weight)
   {
      momentumModuleDataObject.setDesiredSpatialAcceleration(jacobian, taskspaceConstraintData, weight);
   }

   public void setDesiredPointAcceleration(GeometricJacobian jacobian, FramePoint bodyFixedPoint, FrameVector desiredAccelerationWithRespectToBase)
   {
      momentumModuleDataObject.setDesiredPointAcceleration(jacobian, bodyFixedPoint, desiredAccelerationWithRespectToBase);
   }

   public void setDesiredPointAcceleration(GeometricJacobian jacobian, FramePoint bodyFixedPoint, FrameVector desiredAccelerationWithRespectToBase,
           DenseMatrix64F selectionMatrix)
   {
      momentumModuleDataObject.setDesiredPointAcceleration(jacobian, bodyFixedPoint, desiredAccelerationWithRespectToBase, selectionMatrix);
   }

   public void setExternalWrenchToCompensateFor(RigidBody rigidBody, Wrench wrench)
   {
      momentumModuleDataObject.setExternalWrenchToCompensateFor(rigidBody, wrench);
   }

   public MomentumModuleSolution compute(Map<ContactablePlaneBody, ? extends PlaneContactState> contactStates,
                       Map<ContactableCylinderBody, ? extends CylindricalContactState> cylinderContactStates, RobotSide upcomingSupportSide)
           throws NoConvergenceException
   {
      MomentumModuleSolution inactiveSolution = null;
      
      if (TRY_BOTH_AND_COMPARE)
      {
         setMomentumModuleDataObject(inactiveMomentumControlModule, momentumModuleDataObject);
         inactiveSolution = inactiveMomentumControlModule.compute(contactStates, cylinderContactStates, upcomingSupportSide);
      
         momentumModuleSolutionComparer.setMomentumModuleDataObject(momentumModuleDataObject);
         if (this.isUsingOptimizationMomentumControlModule()) momentumModuleSolutionComparer.setOldSolution("Old Solution", inactiveSolution);
         else momentumModuleSolutionComparer.setOptimizationSolution("Optimization Solution", inactiveSolution);
      }
      
      setMomentumModuleDataObject(activeMomentumControlModule, momentumModuleDataObject);
      MomentumModuleSolution activeSolution = activeMomentumControlModule.compute(contactStates, cylinderContactStates, upcomingSupportSide);  
  
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
      momentumControlModule.setDesiredRateOfChangeOfMomentum(desiredRateOfChangeOfMomentumCommand.getMomentumRateOfChangeData());
   }

   private static void setDesiredJointAcceleration(MomentumControlModule momentumControlModule, DesiredJointAccelerationCommand desiredJointAccelerationCommand)
   {
      if (desiredJointAccelerationCommand.getHasWeight())
      {
         momentumControlModule.setDesiredJointAcceleration(desiredJointAccelerationCommand.getJoint(),
                 desiredJointAccelerationCommand.getDesiredAcceleration(), desiredJointAccelerationCommand.getWeight());
      }
      else
      {
         momentumControlModule.setDesiredJointAcceleration(desiredJointAccelerationCommand.getJoint(),
                 desiredJointAccelerationCommand.getDesiredAcceleration());
      }
   }

   private static void setDesiredSpatialAcceleration(MomentumControlModule momentumControlModule,
           DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand)
   {
      if (desiredSpatialAccelerationCommand.getHasWeight())
      {
         momentumControlModule.setDesiredSpatialAcceleration(desiredSpatialAccelerationCommand.getJacobian(),
                 desiredSpatialAccelerationCommand.getTaskspaceConstraintData(), desiredSpatialAccelerationCommand.getWeight());
      }
      else
      {
         momentumControlModule.setDesiredSpatialAcceleration(desiredSpatialAccelerationCommand.getJacobian(),
                 desiredSpatialAccelerationCommand.getTaskspaceConstraintData());
      }
   }


   private static void setDesiredPointAcceleration(MomentumControlModule momentumControlModule, DesiredPointAccelerationCommand desiredPointAccelerationCommand)
   {
      DenseMatrix64F selectionMatrix = desiredPointAccelerationCommand.getSelectionMatrix();

      if (selectionMatrix != null)
      {
         momentumControlModule.setDesiredPointAcceleration(desiredPointAccelerationCommand.getRootToEndEffectorJacobian(),
                 desiredPointAccelerationCommand.getContactPoint(), desiredPointAccelerationCommand.getDesiredAcceleration(), selectionMatrix);
      }

      else
      {
         momentumControlModule.setDesiredPointAcceleration(desiredPointAccelerationCommand.getRootToEndEffectorJacobian(),
                 desiredPointAccelerationCommand.getContactPoint(), desiredPointAccelerationCommand.getDesiredAcceleration());
      }
   }

   private static void setExternalWrenchToCompensateFor(MomentumControlModule momentumControlModule, ExternalWrenchCommand externalWrenchCommand)
   {
      momentumControlModule.setExternalWrenchToCompensateFor(externalWrenchCommand.getRigidBody(), externalWrenchCommand.getWrench());
   }

}
