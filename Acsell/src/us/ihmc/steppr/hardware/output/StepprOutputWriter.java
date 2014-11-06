package us.ihmc.steppr.hardware.output;

import java.util.EnumMap;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.outputs.DRCOutputWriter;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolder;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.steppr.hardware.StepprJoint;
import us.ihmc.steppr.hardware.StepprUtil;
import us.ihmc.steppr.hardware.command.StepprCommand;
import us.ihmc.steppr.hardware.command.StepprJointCommand;
import us.ihmc.steppr.hardware.controllers.StepprStandPrep;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
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
   
   public StepprOutputWriter(DRCRobotModel robotModel)
   {

      SDFFullRobotModel standPrepFullRobotModel = robotModel.createFullRobotModel();
      standPrepJoints = StepprUtil.createJointMap(standPrepFullRobotModel.getOneDoFJoints());

      standPrep.setFullRobotModel(standPrepFullRobotModel);
      registry.addChild(standPrep.getYoVariableRegistry());
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
            
            double tau = wholeBodyControlJoint.getTau() * controlRatio.getDoubleValue() + standPrepJoint.getTau()
                  * (1.0 - controlRatio.getDoubleValue());
            
            
            double kd =  wholeBodyControlJoint.getKd() * controlRatio.getDoubleValue() + standPrepJoint.getKd()
                  * (1.0 - controlRatio.getDoubleValue());
            
            StepprJointCommand jointCommand = command.getStepprJointCommand(joint);
            jointCommand.setTauDesired(tau, rawSensorData);
            jointCommand.setDamping(kd);

         }
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
