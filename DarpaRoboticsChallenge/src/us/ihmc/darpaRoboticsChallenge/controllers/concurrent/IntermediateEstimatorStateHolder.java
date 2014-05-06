package us.ihmc.darpaRoboticsChallenge.controllers.concurrent;

import java.util.ArrayList;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.sensorProcessing.sensors.ForceSensorDataHolder;
import us.ihmc.utilities.ForceSensorDefinition;
import us.ihmc.utilities.GenericCRC32;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.screwTheory.InverseDynamicsJointStateChecksum;
import us.ihmc.utilities.screwTheory.InverseDynamicsJointStateCopier;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SixDoFJoint;

public class IntermediateEstimatorStateHolder
{
   private final GenericCRC32 estimatorChecksumCalculator = new GenericCRC32();
   private final GenericCRC32 controllerChecksumCalculator = new GenericCRC32();
   
   private long estimatorTick;
   private long timestamp;
   private long estimatorClockStartTime;
   private long checksum;

   private final InverseDynamicsJointStateChecksum estimatorChecksum; 
   private final InverseDynamicsJointStateChecksum controllerChecksum; 
   
   private final InverseDynamicsJointStateCopier estimatorToIntermediateCopier;
   private final InverseDynamicsJointStateCopier intermediateToControllerCopier;

   private final ForceSensorDataHolder estimatorForceSensorDataHolder;
   private final ForceSensorDataHolder intermediateForceSensorDataHolder;
   private final ForceSensorDataHolder controllerForceSensorDataHolder;

   public IntermediateEstimatorStateHolder(DRCRobotModel robotModel, RigidBody estimatorRootBody, RigidBody controllerRootBody,
         ArrayList<ForceSensorDefinition> forceSensorDefinitions, ForceSensorDataHolder estimatorForceSensorDataHolder,
         ForceSensorDataHolder controllerForceSensorDataHolder)
   {
      FullRobotModel intermediateModel = robotModel.createFullRobotModel();
      RigidBody intermediateRootBody = intermediateModel.getElevator();

      estimatorChecksum = new InverseDynamicsJointStateChecksum(estimatorRootBody, estimatorChecksumCalculator);
      controllerChecksum = new InverseDynamicsJointStateChecksum(controllerRootBody, controllerChecksumCalculator);
      
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

   public void setFromEstimatorModel(long timestamp, long estimatorTick, long estimatorClockStartTime)
   {
      this.timestamp = timestamp;
      this.estimatorTick = estimatorTick;
      this.estimatorClockStartTime = estimatorClockStartTime;
      
      checksum = calculateEstimatorChecksum();
      estimatorToIntermediateCopier.copy();
      intermediateForceSensorDataHolder.set(estimatorForceSensorDataHolder);
   }

   public void getIntoControllerModel()
   {
      intermediateToControllerCopier.copy();
      controllerForceSensorDataHolder.set(intermediateForceSensorDataHolder);
   }
   
   public long getTimestamp()
   {
      return timestamp;
   }
   
   public long getEstimatorTick()
   {
      return estimatorTick;
   }
   
   public long getEstimatorClockStartTime()
   {
      return estimatorClockStartTime;
   }
   
   private long calculateEstimatorChecksum()
   {
      estimatorChecksumCalculator.reset();
      estimatorChecksum.calculate();
      estimatorForceSensorDataHolder.calculateChecksum(estimatorChecksumCalculator);
      return estimatorChecksumCalculator.getValue();
   }
   
   private long calculateControllerChecksum()
   {
      controllerChecksumCalculator.reset();
      controllerChecksum.calculate();
      controllerForceSensorDataHolder.calculateChecksum(controllerChecksumCalculator);
      return controllerChecksumCalculator.getValue();
   }
   
   public void validate()
   {
      if(checksum != calculateControllerChecksum())
      {
         throw new RuntimeException("Controller checksum doesn't match estimator checksum.");
      }
   }

   public static class Builder implements us.ihmc.concurrent.Builder<IntermediateEstimatorStateHolder>
   {

      private final DRCRobotModel robotModel;
      private final RigidBody estimatorRootJoint;
      private final RigidBody controllerRootJoint;

      private final ArrayList<ForceSensorDefinition> forceSensorDefinitions;
      private final ForceSensorDataHolder estimatorForceSensorDataHolder;
      private final ForceSensorDataHolder controllerForceSensorDataHolder;

      public Builder(DRCRobotModel robotModel, RigidBody estimatorRootJoint, RigidBody controllerRootJoint,
            ArrayList<ForceSensorDefinition> forceSensorDefinitions, ForceSensorDataHolder estimatorForceSensorDataHolder,
            ForceSensorDataHolder controllerForceSensorDataHolder)
      {
         this.robotModel = robotModel;
         this.estimatorRootJoint = estimatorRootJoint;
         this.controllerRootJoint = controllerRootJoint;
         this.forceSensorDefinitions = forceSensorDefinitions;
         this.estimatorForceSensorDataHolder = estimatorForceSensorDataHolder;
         this.controllerForceSensorDataHolder = controllerForceSensorDataHolder;
      }

      @Override
      public IntermediateEstimatorStateHolder newInstance()
      {
         return new IntermediateEstimatorStateHolder(robotModel, estimatorRootJoint, controllerRootJoint, forceSensorDefinitions,
               estimatorForceSensorDataHolder, controllerForceSensorDataHolder);
      }

   }

}
