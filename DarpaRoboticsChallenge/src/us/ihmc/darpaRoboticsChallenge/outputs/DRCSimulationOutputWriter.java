package us.ihmc.darpaRoboticsChallenge.outputs;

import java.util.ArrayList;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFPerfectSimulatedOutputWriter;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.maps.ObjectObjectMap;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.filters.DelayedDoubleYoVariable;

import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.robotController.RawOutputWriter;

public class DRCSimulationOutputWriter extends SDFPerfectSimulatedOutputWriter implements DRCOutputWriter
{
   private static final int TICKS_TO_DELAY = 0;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ObjectObjectMap<OneDoFJoint, DoubleYoVariable> rawJointTorques;
   private final ObjectObjectMap<OneDoFJoint, DelayedDoubleYoVariable> delayedJointTorques;

   private final ArrayList<RawOutputWriter> rawOutputWriters = new ArrayList<RawOutputWriter>();

   double[] prevError;

   public DRCSimulationOutputWriter(SDFRobot robot)
   {
      super(robot);

      rawJointTorques = new ObjectObjectMap<OneDoFJoint, DoubleYoVariable>();
      delayedJointTorques = new ObjectObjectMap<OneDoFJoint, DelayedDoubleYoVariable>();
   }

   @Override
   public void writeAfterController(long timestamp)
   {
      for (int i = 0; i < revoluteJoints.size(); i++)
      {
         Pair<OneDegreeOfFreedomJoint, OneDoFJoint> jointPair = revoluteJoints.get(i);

         OneDegreeOfFreedomJoint pinJoint = jointPair.first();
         OneDoFJoint revoluteJoint = jointPair.second();

         double tau = revoluteJoint.getTau();
         DoubleYoVariable rawJointTorque = rawJointTorques.get(revoluteJoint);
         DelayedDoubleYoVariable delayedJointTorque = delayedJointTorques.get(revoluteJoint);

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
   public void setFullRobotModel(SDFFullRobotModel fullRobotModel, RawJointSensorDataHolderMap rawJointSensorDataHolderMap)
   {
      super.setFullRobotModel(fullRobotModel);

      OneDoFJoint[] joints = fullRobotModel.getOneDoFJoints();
      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint oneDoFJoint = joints[i];
         //         oneDoFJoint.setDampingParameter(DRCRobotDampingParameters.getAtlasDamping(i));

         DoubleYoVariable rawJointTorque = new DoubleYoVariable("raw_tau_" + oneDoFJoint.getName(), registry);
         rawJointTorques.add(oneDoFJoint, rawJointTorque);

         DelayedDoubleYoVariable delayedJointTorque = new DelayedDoubleYoVariable("delayed_tau_" + oneDoFJoint.getName(), "", rawJointTorque, TICKS_TO_DELAY, registry);
         delayedJointTorques.add(oneDoFJoint, delayedJointTorque);
      }

      prevError = new double[revoluteJoints.size()];

   }

   public void addRawOutputWriter(RawOutputWriter rawOutputWriter)
   {
      rawOutputWriters.add(rawOutputWriter);
   }

   @Override
   public void setForceSensorDataHolderForController(ForceSensorDataHolder forceSensorDataHolderForEstimator)
   {
   }

   @Override
   public YoVariableRegistry getControllerYoVariableRegistry()
   {
      return registry;
   }
}
