package us.ihmc.avatar;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextJointData;
import us.ihmc.yoVariables.filters.DelayedYoDouble;
import us.ihmc.robotics.robotController.RawOutputWriter;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.wholeBodyController.DRCOutputProcessor;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class DRCSimulationOutputWriterForControllerThread implements DRCOutputProcessor
{
   private static final int TICKS_TO_DELAY = 0;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final FloatingRootJointRobot robot;
   private final ArrayList<OutputDataSet> revoluteJoints = new ArrayList<>();


   private final ArrayList<RawOutputWriter> rawOutputWriters = new ArrayList<RawOutputWriter>();

   private class OutputDataSet
   {
      private OneDegreeOfFreedomJoint simulatedJoint;
      private JointDesiredOutputBasics jointData;
      private YoDouble rawJointTorque;
      private DelayedYoDouble delayedJointTorque;
   }

   public DRCSimulationOutputWriterForControllerThread(FloatingRootJointRobot robot)
   {
      this.robot = robot;

   }

   @Override
   public void processAfterController(long timestamp)
   {
      for (int i = 0; i < revoluteJoints.size(); i++)
      {

         OutputDataSet data = revoluteJoints.get(i);

         double tau = 0.0;

         if(data.jointData.hasDesiredTorque())
         {
            tau = data.jointData.getDesiredTorque();
         }
         YoDouble rawJointTorque = data.rawJointTorque;
         DelayedYoDouble delayedJointTorque = data.delayedJointTorque;

         if (rawJointTorque != null)
         {
            rawJointTorque.set(tau);
            delayedJointTorque.update();
            tau = delayedJointTorque.getDoubleValue();
         }


         data.simulatedJoint.setTau(tau);
         if(data.jointData.hasStiffness())
         {
            data.simulatedJoint.setKp(data.jointData.getStiffness());
         }
         if(data.jointData.hasDamping())
         {
            data.simulatedJoint.setKd(data.jointData.getDamping());
         }
         if(data.jointData.hasDesiredPosition())
         {
            data.simulatedJoint.setqDesired(data.jointData.getDesiredPosition());
         }
         if(data.jointData.hasDesiredVelocity())
         {
            data.simulatedJoint.setQdDesired(data.jointData.getDesiredVelocity());
         }

      }

      for (int i = 0; i < rawOutputWriters.size(); i++)
      {
         rawOutputWriters.get(i).write();
      }
   }

   @Override
   public void setLowLevelControllerCoreOutput(HumanoidRobotContextJointData estimatedState, JointDesiredOutputListBasics lowLevelControllerOutput)
   {

      for (int i = 0; i < lowLevelControllerOutput.getNumberOfJointsWithDesiredOutput(); i++)
      {
         String jointName = lowLevelControllerOutput.getOneDoFJoint(i).getName();

         OutputDataSet data = new OutputDataSet();
         data.rawJointTorque = new YoDouble("tau_desired_" + jointName, registry);


         data.delayedJointTorque = new DelayedYoDouble("tau_delayed_" + jointName, "", data.rawJointTorque, TICKS_TO_DELAY, registry);

         data.simulatedJoint = robot.getOneDegreeOfFreedomJoint(jointName);
         data.jointData = lowLevelControllerOutput.getJointDesiredOutput(i);

         revoluteJoints.add(data);
      }
   }

   public void addRawOutputWriter(RawOutputWriter rawOutputWriter)
   {
      rawOutputWriters.add(rawOutputWriter);
   }

   @Override
   public void setForceSensorDataHolderForController(ForceSensorDataHolderReadOnly forceSensorDataHolderForEstimator)
   {
   }

   @Override
   public YoRegistry getControllerYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public void initialize()
   {

   }
}
