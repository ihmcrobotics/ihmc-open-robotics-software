package us.ihmc.quadrupedRobotics.stateEstimator.openLoop;

import us.ihmc.quadrupedRobotics.sensorProcessing.SimulatedRobotTimeProvider;
import us.ihmc.quadrupedRobotics.sensorProcessing.simulatedSensors.ControllerOutputMapReadOnly;
import us.ihmc.quadrupedRobotics.stateEstimator.QuadrupedStateEstimator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.JointStateUpdater;

public class OpenLoopQuadrupedStateEstimator implements QuadrupedStateEstimator
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final SimulatedRobotTimeProvider timeProvider;

   private final JointStateUpdater jointStateUpdater;

   public OpenLoopQuadrupedStateEstimator(SimulatedRobotTimeProvider timeProvider, FullInverseDynamicsStructure inverseDynamicsStructure,
         ControllerOutputMapReadOnly controllerOutputReader, YoVariableRegistry registry)
   {
      this.timeProvider = timeProvider;

      jointStateUpdater = new JointStateUpdater(inverseDynamicsStructure, controllerOutputReader, null, this.registry);
      registry.addChild(this.registry);
   }
   
   @Override
   public boolean isFootInContact(RobotQuadrant quadrant)
   {
      // TODO Auto-generated method stub
      return false;
   }

   @Override
   public void doControl()
   {
      timeProvider.doControl();
      jointStateUpdater.updateJointState();
   }

   @Override
   public double getCurrentTime()
   {
      return TimeTools.nanoSecondstoSeconds(timeProvider.getTimestamp());
   }

}
