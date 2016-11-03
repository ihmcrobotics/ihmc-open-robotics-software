package us.ihmc.wanderer.hardware.controllers;

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
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.util.PeriodicRealtimeThreadScheduler;
import us.ihmc.util.PeriodicThreadScheduler;
import us.ihmc.wanderer.hardware.WandererJoint;
import us.ihmc.wanderer.hardware.command.WandererCommand;
import us.ihmc.wanderer.hardware.configuration.WandererNetworkParameters;
import us.ihmc.wanderer.hardware.state.WandererState;
import us.ihmc.wanderer.parameters.WandererRobotModel;

public class WandererSingleThreadedController extends RealtimeThread
{

   public static void startController(WandererController wandererController)
   {
      AcsellSetup.startStreamingData();

      YoVariableRegistry registry = new YoVariableRegistry("wanderer");
      WandererRobotModel robotModel = new WandererRobotModel(true, true);
      PeriodicThreadScheduler scheduler = new PeriodicRealtimeThreadScheduler(45);
      YoVariableServer variableServer = new YoVariableServer(WandererSingleThreadedController.class, scheduler, robotModel.getLogModelProvider(), robotModel.getLogSettings(), 0.01);

      AcsellSetup wandererSetup = new AcsellSetup(variableServer);
      PriorityParameters priority = new PriorityParameters(PriorityParameters.getMaximumPriority());
      WandererSingleThreadedController communicator = new WandererSingleThreadedController(robotModel, priority, variableServer, wandererController, registry);

      variableServer.start();

      wandererSetup.start();
      communicator.start();

      ThreadTools.sleepForever();
   }

   private final RobotVisualizer visualizer;
   private final WandererState state;
   private final WandererController controller;
   private final WandererCommand command;

   private final UDPAcsellOutputWriter outputWriter;

   private final FloatingInverseDynamicsJoint rootJoint;
   private final Quat4d rotation = new Quat4d();
   private final EnumMap<WandererJoint, OneDoFJoint> jointMap = new EnumMap<>(WandererJoint.class);
   private final RawJointSensorDataHolderMap rawSensors;

   private boolean initialized = false;
   private final UDPAcsellStateReader reader;

   private volatile boolean requestStop = false;

   private WandererSingleThreadedController(WandererRobotModel robotModel, PriorityParameters priorityParameters, RobotVisualizer visualizer, WandererController wandererController,
         YoVariableRegistry registry)
   {
      super(priorityParameters);
      this.state = new WandererState(0.001, registry);
      this.reader = new UDPAcsellStateReader(state);

      this.visualizer = visualizer;

      this.command = new WandererCommand(registry);
      this.controller = wandererController;
      this.outputWriter = new UDPAcsellOutputWriter(command);

      
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      rootJoint = fullRobotModel.getRootJoint();

      for (WandererJoint joint : WandererJoint.values)
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

      outputWriter.connect(new WandererNetworkParameters());

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
      for (WandererJoint joint : WandererJoint.values)
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

      for (WandererJoint joint : WandererJoint.values)
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
