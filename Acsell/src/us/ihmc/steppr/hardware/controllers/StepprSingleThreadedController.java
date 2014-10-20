package us.ihmc.steppr.hardware.controllers;

import java.util.EnumMap;

import javax.vecmath.Quat4d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.acsell.parameters.BonoRobotModel;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.robotDataCommunication.YoVariableServer;
import us.ihmc.robotDataCommunication.logger.YoVariableLoggerDispatcher;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolder;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.steppr.hardware.StepprJoint;
import us.ihmc.steppr.hardware.StepprSetup;
import us.ihmc.steppr.hardware.command.StepprCommand;
import us.ihmc.steppr.hardware.command.StepprJointCommand;
import us.ihmc.steppr.hardware.command.UDPStepprOutputWriter;
import us.ihmc.steppr.hardware.configuration.StepprNetworkParameters;
import us.ihmc.steppr.hardware.state.StepprJointState;
import us.ihmc.steppr.hardware.state.StepprState;
import us.ihmc.steppr.hardware.state.StepprStateProcessor;
import us.ihmc.steppr.hardware.state.StepprXSensState;
import us.ihmc.steppr.hardware.state.UDPStepprStateReader;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.humanoidRobot.visualizer.RobotVisualizer;

public class StepprSingleThreadedController implements StepprStateProcessor
{

   public static void startController(StepprController stepprController)
   {
      StepprSetup.startStreamingData();
      
      YoVariableRegistry registry = new YoVariableRegistry("steppr");
      
      StepprState state = new StepprState(registry);      
      YoVariableServer variableServer = new YoVariableServer(5555, 0.01);

      StepprSetup stepprSetup = new StepprSetup(variableServer);
      StepprSingleThreadedController communicator = new StepprSingleThreadedController(state, variableServer, stepprController, registry);

      PriorityParameters priority = new PriorityParameters(PriorityParameters.getMaximumPriority());
      UDPStepprStateReader reader = new UDPStepprStateReader(priority, state, communicator);
      
      variableServer.start();
      YoVariableLoggerDispatcher.requestLogSession(StepprNetworkParameters.LOGGER_HOST, stepprController.getClass().getSimpleName());
      
      stepprSetup.start();
      reader.start();
      
      
      ThreadTools.sleepForever();
   }

   private final RobotVisualizer visualizer;
   private final StepprState state;
   private final StepprController controller;
   private final StepprCommand command;
   
   private final UDPStepprOutputWriter outputWriter;
   
   private final SixDoFJoint rootJoint;
   private final Quat4d rotation = new Quat4d();
   private final EnumMap<StepprJoint, OneDoFJoint> jointMap = new EnumMap<>(StepprJoint.class);
   private final RawJointSensorDataHolderMap rawSensors;

   private boolean initialized = false;
   
   private StepprSingleThreadedController(StepprState state, RobotVisualizer visualizer, StepprController stepprController, YoVariableRegistry registry)
   {
      this.state = state;
      this.visualizer = visualizer;
      
      this.command = new StepprCommand(registry);
      this.controller = stepprController;
      this.outputWriter =  new UDPStepprOutputWriter(command);

      BonoRobotModel robotModel = new BonoRobotModel(true, true);
      SDFFullRobotModel fullRobotModel = robotModel.createFullRobotModel();
      rootJoint = fullRobotModel.getRootJoint();

      for (StepprJoint joint : StepprJoint.values)
      {
         OneDoFJoint oneDoFJoint = fullRobotModel.getOneDoFJointByName(joint.getSdfName());
         if (oneDoFJoint == null)
         {
            throw new RuntimeException("Unknown joint: " + joint.getSdfName());
         }
         jointMap.put(joint, oneDoFJoint);
      }

      this.rawSensors = new RawJointSensorDataHolderMap(fullRobotModel);
      this.controller.setFullRobotModel(fullRobotModel);
      
      if(controller.getYoVariableRegistry() != null)
      {
         registry.addChild(controller.getYoVariableRegistry());
      }
      
      if (visualizer != null)
      {
         visualizer.setMainRegistry(registry, fullRobotModel, null);
      }
      
      outputWriter.connect();

   }

   @Override
   public void process(long timestamp)
   {
      for (StepprJoint joint : StepprJoint.values)
      {
         StepprJointState jointState = state.getJointState(joint);
         OneDoFJoint oneDoFJoint = jointMap.get(joint);
         RawJointSensorDataHolder rawSensor = rawSensors.get(oneDoFJoint);

         oneDoFJoint.setQ(jointState.getQ());
         oneDoFJoint.setQd(jointState.getQd());
         state.updateRawSensorData(joint, rawSensor);
         
      }

      StepprXSensState xSensState = state.getXSensState();
      xSensState.getQuaternion(rotation);
//      rootJoint.setRotation(rotation);
      rootJoint.updateFramesRecursively();
      if(!initialized)
      {
         controller.initialize(timestamp);
         initialized = true;
      }
      
      controller.doControl(timestamp);
      
      if(controller.turnOutputOn())
      {
         command.enableActuators();
      }
      
      for (StepprJoint joint : StepprJoint.values)
      {
         OneDoFJoint oneDoFJoint = jointMap.get(joint);
         RawJointSensorDataHolder rawSensor = rawSensors.get(oneDoFJoint);
         
         StepprJointCommand jointCommand = command.getStepprJointCommand(joint);
         jointCommand.setTauDesired(oneDoFJoint.getTau(), rawSensor);
         jointCommand.setDamping(oneDoFJoint.getKd());
      }

      outputWriter.write();
      
      if (visualizer != null)
      {
         visualizer.update(timestamp);
      }
   }


   @Override
   public void initialize(long timestamp)
   {
      
   }
}
