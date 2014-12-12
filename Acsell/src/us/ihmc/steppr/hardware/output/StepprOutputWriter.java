package us.ihmc.steppr.hardware.output;

import java.util.EnumMap;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolder;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.steppr.hardware.StepprJoint;
import us.ihmc.steppr.hardware.StepprUtil;
import us.ihmc.steppr.hardware.command.StepprCommand;
import us.ihmc.steppr.hardware.command.StepprJointCommand;
import us.ihmc.steppr.hardware.command.UDPStepprOutputWriter;
import us.ihmc.steppr.hardware.controllers.StepprStandPrep;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.wholeBodyController.DRCOutputWriter;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class StepprOutputWriter implements DRCOutputWriter
{
   private final YoVariableRegistry registry = new YoVariableRegistry("StepprOutputWriter");

   private final StepprCommand command = new StepprCommand(registry);

   private EnumMap<StepprJoint, OneDoFJoint> wholeBodyControlJoints;
   private final EnumMap<StepprJoint, OneDoFJoint> standPrepJoints;
   private final StepprStandPrep standPrep = new StepprStandPrep();

   private final DoubleYoVariable controlRatio = new DoubleYoVariable("controlRatio", registry);

   private boolean outputEnabled;
   private final BooleanYoVariable enableOutput = new BooleanYoVariable("enableOutput", registry);

   private RawJointSensorDataHolderMap rawJointSensorDataHolderMap;

   private final UDPStepprOutputWriter outputWriter;
   private final EnumMap<StepprJoint, DoubleYoVariable> yoTauSpring = new EnumMap<StepprJoint, DoubleYoVariable>(StepprJoint.class);

   public StepprOutputWriter(DRCRobotModel robotModel)
   {

      SDFFullRobotModel standPrepFullRobotModel = robotModel.createFullRobotModel();
      standPrepJoints = StepprUtil.createJointMap(standPrepFullRobotModel.getOneDoFJoints());

      outputWriter = new UDPStepprOutputWriter(command);

      standPrep.setFullRobotModel(standPrepFullRobotModel);
      registry.addChild(standPrep.getYoVariableRegistry());

      outputWriter.connect();

      yoTauSpring.put(StepprJoint.LEFT_HIP_X, new DoubleYoVariable(StepprJoint.LEFT_HIP_X.getSdfName() + "_tauSpringCorrection", registry));
      yoTauSpring.put(StepprJoint.RIGHT_HIP_X, new DoubleYoVariable(StepprJoint.RIGHT_HIP_X.getSdfName() + "_tauSpringCorrection", registry));

   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void writeAfterController(long timestamp)
   {
      if (!outputEnabled)
      {
         if (enableOutput.getBooleanValue())
         {
            standPrep.initialize(timestamp);
            command.enableActuators();
            controlRatio.set(0.0);
            outputEnabled = true;
         }
      }
      else
      {

         for (StepprJoint joint : StepprJoint.values)
         {
            OneDoFJoint wholeBodyControlJoint = wholeBodyControlJoints.get(joint);
            OneDoFJoint standPrepJoint = standPrepJoints.get(joint);
            RawJointSensorDataHolder rawSensorData = rawJointSensorDataHolderMap.get(wholeBodyControlJoint);

            standPrepJoint.setQ(rawSensorData.getQ_raw());
            standPrepJoint.setQd(rawSensorData.getQd_raw());
         }

         standPrep.doControl(timestamp);

         for (StepprJoint joint : StepprJoint.values)
         {
            OneDoFJoint wholeBodyControlJoint = wholeBodyControlJoints.get(joint);
            OneDoFJoint standPrepJoint = standPrepJoints.get(joint);
            RawJointSensorDataHolder rawSensorData = rawJointSensorDataHolderMap.get(wholeBodyControlJoint);

            double tau = wholeBodyControlJoint.getTau() * controlRatio.getDoubleValue() + standPrepJoint.getTau() * (1.0 - controlRatio.getDoubleValue());

            double kd = wholeBodyControlJoint.getKd() * controlRatio.getDoubleValue() + standPrepJoint.getKd() * (1.0 - controlRatio.getDoubleValue());

            StepprJointCommand jointCommand = command.getStepprJointCommand(joint);

            double tauSpring = 0;
            if (yoTauSpring.get(joint) != null)
            {
               tauSpring = calcSpringTorque(joint, wholeBodyControlJoint.getQ());
               yoTauSpring.get(joint).set(tauSpring);
            }

//            jointCommand.setTauDesired(tau-tauSpring, rawSensorData); //use when Springs are installed
            jointCommand.setTauDesired(tau, rawSensorData);
            jointCommand.setDamping(kd);

         }
      }

      outputWriter.write();

   }

   private double calcSpringTorque(StepprJoint joint, double q)
   {
      switch (joint)
      {
      case LEFT_HIP_X:
         return 550 * Math.max(-0.05 - q, 0);
      case RIGHT_HIP_X:
         return -550 * Math.max(q - 0.05, 0);
      default:
         return 0;
      }
   }

   @Override
   public void setFullRobotModel(SDFFullRobotModel controllerModel, RawJointSensorDataHolderMap rawJointSensorDataHolderMap)
   {
      wholeBodyControlJoints = StepprUtil.createJointMap(controllerModel.getOneDoFJoints());
      this.rawJointSensorDataHolderMap = rawJointSensorDataHolderMap;
   }

   @Override
   public void setForceSensorDataHolderForController(ForceSensorDataHolder forceSensorDataHolderForController)
   {

   }

   @Override
   public YoVariableRegistry getControllerYoVariableRegistry()
   {
      return registry;
   }

}
