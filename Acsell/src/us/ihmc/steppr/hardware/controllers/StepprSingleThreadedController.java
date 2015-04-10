package us.ihmc.steppr.hardware.controllers;

import java.io.IOException;
import java.util.EnumMap;

import javax.vecmath.Quat4d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.acsell.parameters.BonoRobotModel;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeMemory;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.robotDataCommunication.YoVariableServer;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolder;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.steppr.hardware.StepprJoint;
import us.ihmc.steppr.hardware.StepprSetup;
import us.ihmc.steppr.hardware.command.StepprCommand;
import us.ihmc.steppr.hardware.command.StepprJointCommand;
import us.ihmc.steppr.hardware.command.UDPStepprOutputWriter;
import us.ihmc.steppr.hardware.state.StepprJointState;
import us.ihmc.steppr.hardware.state.StepprState;
import us.ihmc.steppr.hardware.state.StepprXSensState;
import us.ihmc.steppr.hardware.state.UDPStepprStateReader;
import us.ihmc.util.PeriodicRealtimeThreadScheduler;
import us.ihmc.util.PeriodicThreadScheduler;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.humanoidRobot.visualizer.RobotVisualizer;

public class StepprSingleThreadedController extends RealtimeThread
{

   public static void startController(StepprController stepprController)
   {
      StepprSetup.startStreamingData();

      YoVariableRegistry registry = new YoVariableRegistry("steppr");
      BonoRobotModel robotModel = new BonoRobotModel(true, true);
      PeriodicThreadScheduler scheduler = new PeriodicRealtimeThreadScheduler(45);
      YoVariableServer variableServer = new YoVariableServer(StepprSingleThreadedController.class, scheduler, robotModel.getLogModelProvider(), robotModel.getLogSettings(), 0.01);

      StepprSetup stepprSetup = new StepprSetup(variableServer);
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

   private final UDPStepprOutputWriter outputWriter;

   private final SixDoFJoint rootJoint;
   private final Quat4d rotation = new Quat4d();
   private final EnumMap<StepprJoint, OneDoFJoint> jointMap = new EnumMap<>(StepprJoint.class);
   private final RawJointSensorDataHolderMap rawSensors;

   private boolean initialized = false;
   private final UDPStepprStateReader reader;

   private volatile boolean requestStop = false;

   private StepprSingleThreadedController(BonoRobotModel robotModel, PriorityParameters priorityParameters, RobotVisualizer visualizer, StepprController stepprController,
         YoVariableRegistry registry)
   {
      super(priorityParameters);
      this.state = new StepprState(0.001, registry);
      this.reader = new UDPStepprStateReader(state);

      this.visualizer = visualizer;

      this.command = new StepprCommand(registry);
      this.controller = stepprController;
      this.outputWriter = new UDPStepprOutputWriter(command);

      
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

      if (controller.getYoVariableRegistry() != null)
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

         StepprJointCommand jointCommand = command.getStepprJointCommand(joint);
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
