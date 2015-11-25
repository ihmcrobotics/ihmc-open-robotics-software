package us.ihmc.quadrupedRobotics.stateEstimator.kinematicsBased;

import us.ihmc.quadrupedRobotics.sensorProcessing.sensorProcessors.FootContactStateInterface;
import us.ihmc.quadrupedRobotics.sensorProcessing.simulatedSensors.SDFQuadrupedPerfectSimulatedSensor;
import us.ihmc.quadrupedRobotics.stateEstimator.QuadrupedStateEstimator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.JointStateUpdater;

public class QuadrupedKinematicsBasedStateEstimator implements QuadrupedStateEstimator
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   
   private final JointStateUpdater jointStateUpdater;
   private final FootContactStateInterface footSwitchUpdater;


   public QuadrupedKinematicsBasedStateEstimator(FullInverseDynamicsStructure inverseDynamicsStructure, SDFQuadrupedPerfectSimulatedSensor sensorOutputMapReadOnly, FootContactStateInterface footSwitchUpdater)
   {
      jointStateUpdater = new JointStateUpdater(inverseDynamicsStructure, sensorOutputMapReadOnly, null, registry);
      
      this.footSwitchUpdater = footSwitchUpdater; 

   }

   @Override
   public boolean isFootInContact(RobotQuadrant quadrant)
   {
      return footSwitchUpdater.isFootInContactWithGround(quadrant);
   }

   public void initialize()
   {
      jointStateUpdater.initialize();
   }
   
   @Override
   public void doControl()
   {
      jointStateUpdater.updateJointState();
   }
}
