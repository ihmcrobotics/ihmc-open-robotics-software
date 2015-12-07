package us.ihmc.quadrupedRobotics.stateEstimator.openLoop;

import us.ihmc.quadrupedRobotics.sensorProcessing.simulatedSensors.ControllerOutputMapReadOnly;
import us.ihmc.quadrupedRobotics.stateEstimator.QuadrupedStateEstimator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.JointStateUpdater;

public class OpenLoopQuadrupedStateEstimator implements QuadrupedStateEstimator
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final DoubleYoVariable yoTime;

   private final JointStateUpdater jointStateUpdater;

   public OpenLoopQuadrupedStateEstimator(DoubleYoVariable yoTime, FullInverseDynamicsStructure inverseDynamicsStructure,
         ControllerOutputMapReadOnly controllerOutputReader, YoVariableRegistry registry)
   {
      this.yoTime = yoTime;

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
      jointStateUpdater.updateJointState();
   }

   @Override
   public double getCurrentTime()
   {
      return yoTime.getDoubleValue();
   }

   @Override
   public void initialize()
   {
      // TODO Auto-generated method stub
      
   }

}
