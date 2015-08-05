package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

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
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoUtilities.time.ExecutionTimer;


public class MomentumControlModuleBridge implements MomentumControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry("MomentumControlModuleBridge");

   private static final boolean LISTEN_IN_ON_SOLVER = false;
   private static final boolean TRY_ALL_AND_COMPARE = false;
   private static final boolean SHOW_MOMENTUM_MODULE_GUI =  false; //true;
   
   private final MomentumModuleSolutionComparer momentumModuleSolutionComparer;
   private final MomentumModuleGUI momentumModuleGUI;

   public enum MomentumControlModuleType 
   {
     OPT_NULLSPACE, OLD;
   };

   
   private final EnumMap<MomentumControlModuleType, MomentumControlModule> momentumControlModules = 
         new EnumMap<MomentumControlModuleType, MomentumControlModule>(MomentumControlModuleType.class);
   private final EnumYoVariable<MomentumControlModuleType> activeMomentumControlModuleType = 
         new EnumYoVariable<MomentumControlModuleType>("activeMomentumControlModuleType", registry, MomentumControlModuleType.class);
   private final EnumYoVariable<MomentumControlModuleType> referenceMomentumControlModuleType = 
         new EnumYoVariable<MomentumControlModuleType>("referenceMomentumControlModuleType", "null is TRY_ALL_TO_COMPARE=false", registry, MomentumControlModuleType.class, true);
   private final EnumYoVariable<MomentumControlModuleType> requestMomentumControlModuleType=
           new EnumYoVariable<MomentumControlModuleBridge.MomentumControlModuleType>("requestedMomentumControlModuleType","null is making no request", registry, MomentumControlModuleType.class, true);
   private final ExecutionTimer momentumControlModuleTimer = new ExecutionTimer("momentumControlModuleTimer", 0.0, registry);


   private final BooleanYoVariable ignoreZDDot = new BooleanYoVariable("ignoreZDDot", registry);
   
   private MomentumControlModule activeMomentumControlModule, referenceMomentumControlModule;

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
      
      requestMomentumControlModuleType.set(null);

      
      //add controllers
      momentumControlModules.put(MomentumControlModuleType.OPT_NULLSPACE, optimizationMomentumControlModule);
      momentumControlModules.put(MomentumControlModuleType.OLD, oldMomentumControlModule);

      setMomentumControlModuleToUse(MomentumControlModuleType.OPT_NULLSPACE);

      if (TRY_ALL_AND_COMPARE) 
      {
         momentumModuleSolutionComparer = new MomentumModuleSolutionComparer(centerOfMassFrame, registry);
         setMomentumControlModuleToUse(MomentumControlModuleType.OLD);
         for(MomentumControlModuleType m : momentumControlModules.keySet())
         {
           if(momentumControlModules.get(m)==null) 
           {
              System.err.println(getClass().getSimpleName()+": Not all controllers are valid for comparison."+
                    "controller "+ m.name() + " is null."+
                    "please fix the controllers or disable TRY_ALL_AND_COMAPRE");
           }
         }
      }
      else 
      {
         momentumModuleSolutionComparer = null;
      }

      parentRegistry.addChild(registry);
   }

   
   public void setMomentumControlModuleToUse(MomentumControlModuleType momentumControlModuleTypeToUse)
   {
      if(momentumControlModules.get(momentumControlModuleTypeToUse)==null)
      {
         throw new RuntimeException("Requested ControllerModule is null ...");
      }

      MomentumControlModuleType prevModuleType = activeMomentumControlModuleType.getEnumValue();
      activeMomentumControlModule = momentumControlModules.get(momentumControlModuleTypeToUse);
      activeMomentumControlModuleType.set(momentumControlModuleTypeToUse);

      if (TRY_ALL_AND_COMPARE)
      {
         if(prevModuleType==null)
            prevModuleType = activeMomentumControlModuleType.getEnumValue();

         if(referenceMomentumControlModuleType.getEnumValue()!=null && prevModuleType!=activeMomentumControlModuleType.getEnumValue())
         {
            referenceMomentumControlModuleType.set(prevModuleType);
            referenceMomentumControlModule = momentumControlModules.get(prevModuleType);
         }
      }
      else
      {
         referenceMomentumControlModuleType.set(null);
         referenceMomentumControlModule=null;
      }
      
      
      for (MomentumControlModule momentumControlModule : momentumControlModules.values())
      {
         if (momentumControlModule != null)
            momentumControlModule.initialize();
      }
   }

   public void initialize()
   {
      activeMomentumControlModule.initialize();

      if (SHOW_MOMENTUM_MODULE_GUI)
      {
         momentumModuleGUI.initialize();
      }
      
      if (TRY_ALL_AND_COMPARE)
      {
         referenceMomentumControlModule.initialize();
      }
   }

   public void reset()
   {
      if (requestMomentumControlModuleType.getEnumValue()!=null && requestMomentumControlModuleType.getEnumValue() != activeMomentumControlModuleType.getEnumValue())
      {
         try{
                 setMomentumControlModuleToUse(requestMomentumControlModuleType.getEnumValue());
         }
         catch(RuntimeException e)
         {
            System.err.println("Requested ControllerModule is null, revert...");
            requestMomentumControlModuleType.set(activeMomentumControlModuleType.getEnumValue());
            return;
         }
         finally
         {
            requestMomentumControlModuleType.set(null);
         }
      }
      
      activeMomentumControlModule.reset();
      momentumModuleDataObject.reset();
      
      if (SHOW_MOMENTUM_MODULE_GUI)
      {
         allMomentumModuleListener.reset();
         momentumModuleGUI.reset();
      }
      
      if (TRY_ALL_AND_COMPARE)
      {
         referenceMomentumControlModule.reset();
      }
   }

   public void resetGroundReactionWrenchFilter()
   {
      activeMomentumControlModule.resetGroundReactionWrenchFilter();
      
      if (TRY_ALL_AND_COMPARE)
      {
         referenceMomentumControlModule.resetGroundReactionWrenchFilter();
      }
   }

   public void setDesiredRateOfChangeOfMomentum(DesiredRateOfChangeOfMomentumCommand desiredRateOfChangeOfMomentumCommand)
   {
      MomentumRateOfChangeData momentumRateOfChangeData = desiredRateOfChangeOfMomentumCommand.getMomentumRateOfChangeData();
      DenseMatrix64F momentumMultipliers = momentumRateOfChangeData.getMomentumMultipliers();
//      DenseMatrix64F momentumSubspace = momentumRateOfChangeData.getMomentumSubspace();
      
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
      if (allMomentumModuleListener != null) 
         allMomentumModuleListener.desiredRateOfChangeOfMomentumWasSet(desiredRateOfChangeOfMomentumCommand);
   }

   public void setDesiredJointAcceleration(DesiredJointAccelerationCommand desiredJointAccelerationCommand)
   {
      momentumModuleDataObject.setDesiredJointAcceleration(desiredJointAccelerationCommand);
      if (allMomentumModuleListener != null) 
         allMomentumModuleListener.desiredJointAccelerationWasSet(desiredJointAccelerationCommand);
   }

   public void setDesiredSpatialAcceleration(DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand)
   {
      momentumModuleDataObject.setDesiredSpatialAcceleration(desiredSpatialAccelerationCommand);
      if (allMomentumModuleListener != null) 
         allMomentumModuleListener.desiredSpatialAccelerationWasSet(desiredSpatialAccelerationCommand);
   }
   
   public void setDesiredPointAcceleration(DesiredPointAccelerationCommand desiredPointAccelerationCommand)
   {
      momentumModuleDataObject.setDesiredPointAcceleration(desiredPointAccelerationCommand);
      if (allMomentumModuleListener != null) 
         allMomentumModuleListener.desiredPointAccelerationWasSet(desiredPointAccelerationCommand);
   }

   public void setExternalWrenchToCompensateFor(RigidBody rigidBody, Wrench wrench)
   {
      momentumModuleDataObject.setExternalWrenchToCompensateFor(rigidBody, wrench);
   }
   
   public void setFootCoPControlData(RobotSide side, ReferenceFrame frame)
   {
      activeMomentumControlModule.setFootCoPControlData(side, frame);
   }

   public MomentumModuleSolution compute(Map<ContactablePlaneBody, ? extends PlaneContactState> contactStates, RobotSide upcomingSupportSide)
           throws MomentumControlModuleException
   {
      //Note sensor data may be updated in other process, so the comparison of two computations may not be exact
      //Here, we put the two setMomentumModuleDataObject closer intime so both module get closer input....
      setMomentumModuleDataObject(activeMomentumControlModule, momentumModuleDataObject);
      if(TRY_ALL_AND_COMPARE)
         setMomentumModuleDataObject(referenceMomentumControlModule, momentumModuleDataObject);

      momentumControlModuleTimer.startMeasurement();
      MomentumModuleSolution activeSolution = activeMomentumControlModule.compute(contactStates, upcomingSupportSide);  
      momentumControlModuleTimer.stopMeasurement();

      if (TRY_ALL_AND_COMPARE)
      {
         momentumModuleSolutionComparer.setActiveSolution("Optimization Solution", activeSolution);

         MomentumModuleSolution referenceSolution = referenceMomentumControlModule.compute(contactStates, upcomingSupportSide);
         momentumModuleSolutionComparer.setReferenceSolution("Reference Solution", referenceSolution);

         momentumModuleSolutionComparer.setMomentumModuleDataObject(momentumModuleDataObject);
         momentumModuleSolutionComparer.displayComparison();
      }

      if (allMomentumModuleListener != null)
      {
         allMomentumModuleListener.momentumModuleSolutionWasComputed(activeSolution);
      }
      
      if (SHOW_MOMENTUM_MODULE_GUI)
      {
         momentumModuleGUI.update(allMomentumModuleListener);
//         momentumModuleGUI.setDesiredsAndSolution(momentumModuleDataObject, activeSolution);
      }
      
      return activeSolution;
   }

   private static void setMomentumModuleDataObject(MomentumControlModule momentumControlModule, MomentumModuleDataObject momentumModuleDataObject)
   {
      ArrayList<DesiredRateOfChangeOfMomentumCommand> desiredRateOfChangeOfMomentumCommands = momentumModuleDataObject.getDesiredRateOfChangeOfMomentumCommands();
      for(int i = 0; i < desiredRateOfChangeOfMomentumCommands.size(); i++)
      {
         DesiredRateOfChangeOfMomentumCommand desiredRateOfChangeOfMomentumCommand = desiredRateOfChangeOfMomentumCommands.get(i);
         setDesiredRateOfChangeOfMomentum(momentumControlModule, desiredRateOfChangeOfMomentumCommand);
      }

      ArrayList<DesiredJointAccelerationCommand> desiredJointAccelerationCommands = momentumModuleDataObject.getDesiredJointAccelerationCommands();
      for(int i = 0; i < desiredJointAccelerationCommands.size(); i++)
      {
         DesiredJointAccelerationCommand desiredJointAccelerationCommand = desiredJointAccelerationCommands.get(i);
         setDesiredJointAcceleration(momentumControlModule, desiredJointAccelerationCommand);
      }

      ArrayList<DesiredSpatialAccelerationCommand> desiredSpatialAccelerationCommands = momentumModuleDataObject.getDesiredSpatialAccelerationCommands();
      for(int i = 0; i < desiredSpatialAccelerationCommands.size(); i++)
      {
         DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand = desiredSpatialAccelerationCommands.get(i);
         setDesiredSpatialAcceleration(momentumControlModule, desiredSpatialAccelerationCommand);
      }

      ArrayList<DesiredPointAccelerationCommand> desiredPointAccelerationCommands = momentumModuleDataObject.getDesiredPointAccelerationCommands();
      for(int i = 0; i < desiredPointAccelerationCommands.size(); i++)
      {
         DesiredPointAccelerationCommand desiredPointAccelerationCommand = desiredPointAccelerationCommands.get(i);
         setDesiredPointAcceleration(momentumControlModule, desiredPointAccelerationCommand);
      }

      ArrayList<ExternalWrenchCommand> externalWrenchCommands = momentumModuleDataObject.getExternalWrenchCommands();
      for(int i = 0; i < externalWrenchCommands.size(); i++)
      {
         ExternalWrenchCommand externalWrenchCommand = externalWrenchCommands.get(i);
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
