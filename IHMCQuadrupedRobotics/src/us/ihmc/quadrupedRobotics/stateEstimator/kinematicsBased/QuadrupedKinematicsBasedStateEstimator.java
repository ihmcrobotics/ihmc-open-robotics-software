package us.ihmc.quadrupedRobotics.stateEstimator.kinematicsBased;

import us.ihmc.quadrupedRobotics.sensorProcessing.simulatedSensors.SDFQuadrupedPerfectSimulatedSensor;
import us.ihmc.quadrupedRobotics.stateEstimator.QuadrupedStateEstimator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.JointStateUpdater;

public class QuadrupedKinematicsBasedStateEstimator implements QuadrupedStateEstimator
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   
   private final JointStateUpdater jointStateUpdater;
   private final FootSwitchUpdater footSwitchUpdater;
   private QuadrantDependentList<BooleanYoVariable> footContactSwitches = new QuadrantDependentList<>();

   public QuadrupedKinematicsBasedStateEstimator(FullInverseDynamicsStructure inverseDynamicsStructure, SDFQuadrupedPerfectSimulatedSensor sensorOutputMapReadOnly)
   {
      jointStateUpdater = new JointStateUpdater(inverseDynamicsStructure, sensorOutputMapReadOnly, null, registry);
      
      for(RobotQuadrant quadrant : RobotQuadrant.values())
      {
         String name = getClass().getSimpleName() + quadrant.getCamelCaseNameForMiddleOfExpression() + "footSwitch";
         BooleanYoVariable footSwitch = new BooleanYoVariable(name, registry);
         footContactSwitches.put(quadrant, footSwitch);
      }
      
      footSwitchUpdater = new FootSwitchUpdater(footContactSwitches , sensorOutputMapReadOnly, registry);

   }

   @Override
   public boolean isFootInContact(RobotQuadrant quadrant)
   {
      return footContactSwitches.get(quadrant).getBooleanValue();
   }

   public void initialize()
   {
      jointStateUpdater.initialize();
      footSwitchUpdater.initialize();
   }
   
   
   public void doControl()
   {
      jointStateUpdater.updateJointState();
      footSwitchUpdater.updateFootSwitchState();
   }
   
}
