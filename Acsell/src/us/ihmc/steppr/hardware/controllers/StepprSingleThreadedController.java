package us.ihmc.steppr.hardware.controllers;

import java.io.IOException;
import java.util.EnumMap;

import javax.vecmath.Quat4d;

import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.visualizer.RobotVisualizer;
import us.ihmc.acsell.hardware.AcsellSetup;
import us.ihmc.acsell.hardware.command.AcsellJointCommand;
import us.ihmc.acsell.hardware.command.UDPAcsellOutputWriter;
import us.ihmc.acsell.hardware.state.AcsellJointState;
import us.ihmc.acsell.hardware.state.AcsellXSensState;
import us.ihmc.acsell.hardware.state.UDPAcsellStateReader;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeMemory;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolder;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.steppr.hardware.StepprJoint;
import us.ihmc.steppr.hardware.command.StepprCommand;
import us.ihmc.steppr.hardware.configuration.StepprNetworkParameters;
import us.ihmc.steppr.hardware.state.StepprState;
import us.ihmc.steppr.parameters.BonoRobotModel;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.util.PeriodicRealtimeThreadScheduler;
import us.ihmc.util.PeriodicThreadScheduler;

public class StepprSingleThreadedController extends RealtimeThread
{

   public static void startController(StepprController stepprController)
   {
      AcsellSetup.startStreamingData();

      YoVariableRegistry registry = new YoVariableRegistry("steppr");
      BonoRobotModel robotModel = new BonoRobotModel(true, true);
      PeriodicThreadScheduler scheduler = new PeriodicRealtimeThreadScheduler(45);
      YoVariableServer variableServer = new YoVariableServer(StepprSingleThreadedController.class, scheduler, robotModel.getLogModelProvider(), robotModel.getLogSettings(), 0.01);

      AcsellSetup stepprSetup = new AcsellSetup(variableServer);
      PriorityParameters priority = new PriorityParameters(PriorityParameters.getMaximumPriority());
      StepprSingleThreadedController communicator = new StepprSingleThreadedController(robotModel, priority, variableServer, stepprController, registry);

      variableServer.start();

      stepprSetup.start();
      communicator.start();

      ThreadTools.sleepForever();
   }

   private final RobotVisualizer visualizer;
   private final StepprState state;
   private final StepprController controller;
   private final StepprCommand command;

   private final UDPAcsellOutputWriter outputWriter;

   private final FloatingInverseDynamicsJoint rootJoint;
   private final Quat4d rotation = new Quat4d();
   private final EnumMap<StepprJoint, OneDoFJoint> jointMap = new EnumMap<>(StepprJoint.class);
   private final RawJointSensorDataHolderMap rawSensors;

   private boolean initialized = false;
   private final UDPAcsellStateReader reader;

   private volatile boolean requestStop = false;

   private StepprSingleThreadedController(BonoRobotModel robotModel, PriorityParameters priorityParameters, RobotVisualizer visualizer, StepprController stepprController,
         YoVariableRegistry registry)
   {
      super(priorityParameters);
      this.state = new StepprState(0.001, registry);
      this.reader = new UDPAcsellStateReader(state);

      this.visualizer = visualizer;

      this.command = new StepprCommand(registry);
      this.controller = stepprController;
      this.outputWriter = new UDPAcsellOutputWriter(command);

      
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
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

      if (controller.getYoVariableRegistry() != null)
      {
         registry.addChild(controller.getYoVariableRegistry());
      }

      if (visualizer != null)
      {
         visualizer.setMainRegistry(registry, fullRobotModel, null);
      }

      outputWriter.connect(new StepprNetworkParameters());

   }

   @Override
   public void run()
   {
      try
      {
         reader.connect();
      }
      catch (IOException e)
      {
         reader.disconnect();

         throw new RuntimeException(e);
      }

      System.gc();
      System.gc();

      RealtimeMemory.lock();
      try
      {

         while (!requestStop)
         {
            long receiveTime = reader.receive();
            if (receiveTime != -1)
            {
               process(receiveTime);
            }
         }
      }
      catch (IOException e)
      {
         reader.disconnect();

         throw new RuntimeException(e);
      }

      reader.disconnect();
   }

   public void process(long timestamp)
   {
      for (StepprJoint joint : StepprJoint.values)
      {
         AcsellJointState jointState = state.getJointState(joint);
         OneDoFJoint oneDoFJoint = jointMap.get(joint);
         RawJointSensorDataHolder rawSensor = rawSensors.get(oneDoFJoint);

         oneDoFJoint.setQ(jointState.getQ());
         oneDoFJoint.setQd(jointState.getQd());
         state.updateRawSensorData(joint, rawSensor);

      }

      AcsellXSensState xSensState = state.getXSensState();
      xSensState.getQuaternion(rotation);
      //      rootJoint.setRotation(rotation);
      rootJoint.updateFramesRecursively();
      if (!initialized)
      {
         controller.initialize(timestamp);
         initialized = true;
      }

      controller.doControl(timestamp);

      if (controller.turnOutputOn())
      {
         command.enableActuators();
      }

      for (StepprJoint joint : StepprJoint.values)
      {
         OneDoFJoint oneDoFJoint = jointMap.get(joint);
         RawJointSensorDataHolder rawSensor = rawSensors.get(oneDoFJoint);

         AcsellJointCommand jointCommand = command.getAcsellJointCommand(joint);
         jointCommand.setTauDesired(oneDoFJoint.getTau(), 0.0, rawSensor);
         jointCommand.setDamping(oneDoFJoint.getKd());
      }

      outputWriter.write();

      if (visualizer != null)
      {
         visualizer.update(timestamp);
      }
   }

}
