package us.ihmc.avatar;

import java.util.ArrayList;
import java.util.LinkedHashMap;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.math.filters.DelayedYoDouble;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.robotController.RawOutputWriter;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.simulationToolkit.outputWriters.PerfectSimulatedOutputWriter;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.wholeBodyController.DRCOutputWriter;

public class DRCSimulationOutputWriter extends PerfectSimulatedOutputWriter implements DRCOutputWriter
{
   private static final int TICKS_TO_DELAY = 0;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final LinkedHashMap<OneDoFJoint, YoDouble> rawJointTorques = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, DelayedYoDouble> delayedJointTorques = new LinkedHashMap<>();

   private final ArrayList<RawOutputWriter> rawOutputWriters = new ArrayList<RawOutputWriter>();

   public DRCSimulationOutputWriter(FloatingRootJointRobot robot)
   {
      super(robot);
   }

   @Override
   public void writeAfterController(long timestamp)
   {
      for (int i = 0; i < revoluteJoints.size(); i++)
      {
         ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint> jointPair = revoluteJoints.get(i);

         OneDegreeOfFreedomJoint pinJoint = jointPair.getLeft();
         OneDoFJoint revoluteJoint = jointPair.getRight();

         double tau = revoluteJoint.getTau();
         YoDouble rawJointTorque = rawJointTorques.get(revoluteJoint);
         DelayedYoDouble delayedJointTorque = delayedJointTorques.get(revoluteJoint);

         if (rawJointTorque != null)
         {
            rawJointTorque.set(tau);
            delayedJointTorque.update();
            tau = delayedJointTorque.getDoubleValue();
         }

         pinJoint.setTau(tau);
         pinJoint.setKp(revoluteJoint.getKp());
         pinJoint.setKd(revoluteJoint.getKd());
         pinJoint.setqDesired(revoluteJoint.getqDesired());
         pinJoint.setQdDesired(revoluteJoint.getQdDesired());

      }

      for (int i = 0; i < rawOutputWriters.size(); i++)
      {
         rawOutputWriters.get(i).write();
      }
   }

   @Override
   public void setFullRobotModel(FullHumanoidRobotModel fullRobotModel, RawJointSensorDataHolderMap rawJointSensorDataHolderMap)
   {
      super.setFullRobotModel(fullRobotModel);

      OneDoFJoint[] joints = fullRobotModel.getOneDoFJoints();
      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint oneDoFJoint = joints[i];
         String jointName = oneDoFJoint.getName();

         YoDouble rawJointTorque = new YoDouble("tau_desired_" + jointName, registry);
         rawJointTorques.put(oneDoFJoint, rawJointTorque);

         DelayedYoDouble delayedJointTorque = new DelayedYoDouble("tau_delayed_" + jointName, "", rawJointTorque, TICKS_TO_DELAY, registry);
         delayedJointTorques.put(oneDoFJoint, delayedJointTorque);
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
   public YoVariableRegistry getControllerYoVariableRegistry()
   {
      return registry;
   }
}
