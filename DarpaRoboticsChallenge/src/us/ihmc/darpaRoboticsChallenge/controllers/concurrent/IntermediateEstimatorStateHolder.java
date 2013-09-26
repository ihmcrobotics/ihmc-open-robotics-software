package us.ihmc.darpaRoboticsChallenge.controllers.concurrent;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModelFactory;
import us.ihmc.sensorProcessing.sensors.ForceSensorDataHolder;
import us.ihmc.utilities.ForceSensorDefinition;
import us.ihmc.utilities.screwTheory.InverseDynamicsJointStateCopier;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SixDoFJoint;

public class IntermediateEstimatorStateHolder
{

   private long timestamp;
   
   private final InverseDynamicsJointStateCopier estimatorToIntermediateCopier;
   private final InverseDynamicsJointStateCopier intermediateToControllerCopier;

   private final ForceSensorDataHolder estimatorForceSensorDataHolder;
   private final ForceSensorDataHolder intermediateForceSensorDataHolder;
   private final ForceSensorDataHolder controllerForceSensorDataHolder;

   public IntermediateEstimatorStateHolder(FullRobotModelFactory intermediateFactory, RigidBody estimatorRootBody, RigidBody controllerRootBody,
         ArrayList<ForceSensorDefinition> forceSensorDefinitions, ForceSensorDataHolder estimatorForceSensorDataHolder,
         ForceSensorDataHolder controllerForceSensorDataHolder)
   {
      FullRobotModel intermediateModel = intermediateFactory.create();
      RigidBody intermediateRootBody = intermediateModel.getElevator();

      estimatorToIntermediateCopier = new InverseDynamicsJointStateCopier(estimatorRootBody, intermediateRootBody);
      intermediateToControllerCopier = new InverseDynamicsJointStateCopier(intermediateRootBody, controllerRootBody);

      this.estimatorForceSensorDataHolder = estimatorForceSensorDataHolder;
      this.intermediateForceSensorDataHolder = new ForceSensorDataHolder(forceSensorDefinitions, intermediateModel.getRootJoint());
      this.controllerForceSensorDataHolder = controllerForceSensorDataHolder;
   }

   public void setForceSensorDefinitions(ArrayList<ForceSensorDefinition> forceSensorDefinitions, SixDoFJoint rootJoint,
         ForceSensorDataHolder estimatorForceSensorDataHolder, ForceSensorDataHolder controllerForceSensorDataHolder)
   {
   }

   public void setFromEstimatorModel(long timestamp)
   {
      this.timestamp = timestamp;
      estimatorToIntermediateCopier.copy();
      intermediateForceSensorDataHolder.set(estimatorForceSensorDataHolder);
   }

   public long getIntoControllerModel()
   {
      intermediateToControllerCopier.copy();
      controllerForceSensorDataHolder.set(intermediateForceSensorDataHolder);
      return timestamp;
   }

   public static class Builder implements us.ihmc.concurrent.Builder<IntermediateEstimatorStateHolder>
   {

      private final FullRobotModelFactory intermediateFactory;
      private final RigidBody estimatorRootJoint;
      private final RigidBody controllerRootJoint;

      private final ArrayList<ForceSensorDefinition> forceSensorDefinitions;
      private final ForceSensorDataHolder estimatorForceSensorDataHolder;
      private final ForceSensorDataHolder controllerForceSensorDataHolder;

      public Builder(FullRobotModelFactory intermediateFactory, RigidBody estimatorRootJoint, RigidBody controllerRootJoint,
            ArrayList<ForceSensorDefinition> forceSensorDefinitions, ForceSensorDataHolder estimatorForceSensorDataHolder,
            ForceSensorDataHolder controllerForceSensorDataHolder)
      {
         this.intermediateFactory = intermediateFactory;
         this.estimatorRootJoint = estimatorRootJoint;
         this.controllerRootJoint = controllerRootJoint;
         this.forceSensorDefinitions = forceSensorDefinitions;
         this.estimatorForceSensorDataHolder = estimatorForceSensorDataHolder;
         this.controllerForceSensorDataHolder = controllerForceSensorDataHolder;
      }

      @Override
      public IntermediateEstimatorStateHolder newInstance()
      {
         return new IntermediateEstimatorStateHolder(intermediateFactory, estimatorRootJoint, controllerRootJoint, forceSensorDefinitions,
               estimatorForceSensorDataHolder, controllerForceSensorDataHolder);
      }

   }

}
