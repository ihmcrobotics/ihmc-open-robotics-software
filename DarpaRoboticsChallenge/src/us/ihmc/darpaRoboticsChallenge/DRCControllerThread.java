package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.corruptors.FullRobotModelCorruptor;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.MomentumBasedControllerFactory;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonHumanoidReferenceFramesVisualizer;
import us.ihmc.commonWalkingControlModules.sensors.CenterOfMassJacobianUpdater;
import us.ihmc.commonWalkingControlModules.sensors.CommonHumanoidReferenceFramesUpdater;
import us.ihmc.commonWalkingControlModules.sensors.ReferenceFrameUpdater;
import us.ihmc.commonWalkingControlModules.sensors.TwistUpdater;
import us.ihmc.commonWalkingControlModules.visualizer.CommonInertiaElipsoidsVisualizer;
import us.ihmc.commonWalkingControlModules.visualizer.ForceSensorDataVisualizer;
import us.ihmc.darpaRoboticsChallenge.calib.CenterOfMassCalibrationTool;
import us.ihmc.darpaRoboticsChallenge.controllers.concurrent.ThreadDataSynchronizer;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotLidarParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.darpaRoboticsChallenge.outputs.DRCOutputWriter;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.io.streamingData.GlobalDataProducer;
import us.ihmc.utilities.math.TimeTools;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.wholeBodyController.ConstrainedCenterOfMassJacobianEvaluator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.LongYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.humanoidRobot.visualizer.RobotVisualizer;
import us.ihmc.yoUtilities.time.ExecutionTimer;
import us.ihmc.simulationconstructionset.InverseDynamicsMechanismReferenceFrameVisualizer;
import us.ihmc.simulationconstructionset.JointAxisVisualizer;
import us.ihmc.simulationconstructionset.robotController.ModularRobotController;
import us.ihmc.simulationconstructionset.robotController.ModularSensorProcessor;
import us.ihmc.simulationconstructionset.robotController.MultiThreadedRobotControlElement;
import us.ihmc.simulationconstructionset.robotController.RobotController;

public class DRCControllerThread implements MultiThreadedRobotControlElement
{
   private static final boolean CREATE_DYNAMICALLY_CONSISTENT_NULLSPACE_EVALUATOR = false;
   private static final boolean SHOW_INERTIA_GRAPHICS = false;
   private static final boolean SHOW_REFERENCE_FRAMES = false;
   private static final boolean SHOW_JOINTAXIS_ZALIGN_FRAMES = false;
   private static final boolean SHOW_WRIST_SENSOR_WRENCHES = true;

 
   private static final boolean CREATE_COM_CALIBRATION_TOOL = false;
   
   private final YoVariableRegistry registry = new YoVariableRegistry("DRCControllerThread");
   private final RobotVisualizer robotVisualizer;

   private final long controlDTInNS;
   private final long estimatorDTInNS;
   private final long estimatorTicksPerControlTick;
   
   private final DoubleYoVariable controllerTime = new DoubleYoVariable("controllerTime", registry);
   private final BooleanYoVariable firstTick = new BooleanYoVariable("firstTick", registry);
   

   private final SDFFullRobotModel controllerFullRobotModel;
   private final FullRobotModelCorruptor fullRobotModelCorruptor;
   
   private final ReferenceFrames controllerReferenceFrames;

   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private final ForceSensorDataHolder forceSensorDataHolderForController;

   private final ThreadDataSynchronizer threadDataSynchronizer;
   private final DRCOutputWriter outputWriter;
   
   private final RobotController robotController;
   
   private final ExecutionTimer controllerTimer = new ExecutionTimer("controllerTimer", 10.0, registry);
   private final LongYoVariable lastEstimatorStartTime = new LongYoVariable("nextExecutionTime", registry);
   private final LongYoVariable totalDelay = new LongYoVariable("totalDelay", registry);
   private final LongYoVariable expectedEstimatorTick = new LongYoVariable("expectedEstimatorTick", registry);
   private final LongYoVariable controllerLeadsEstimatorTicks = new LongYoVariable("controllerLeadsEstimatorTicks", registry);
   private final LongYoVariable controllerLagsEstimatorTicks = new LongYoVariable("controllerLagsEstimatorTicks", registry);
   
   /*
    * Debug variables
    */
   private final LongYoVariable lastExpectedEstimatorTick = new LongYoVariable("lastExpectedEstimatorTick", registry);
   private final LongYoVariable lastEstimatorTick = new LongYoVariable("lastEstimatorTick", registry);
   private final LongYoVariable lastEstimatorClockStartTime = new LongYoVariable("lastEstimatorClockStartTime", registry);
   private final LongYoVariable lastControllerClockTime = new LongYoVariable("lastControllerClockTime", registry);
   private final LongYoVariable controllerStartTime = new LongYoVariable("controllerStartTime", registry);
   private final LongYoVariable actualControlDT = new LongYoVariable("actualControlDT", registry);
   private final LongYoVariable timePassedSinceEstimator = new LongYoVariable("timePassedSinceEstimator", registry);
   private final LongYoVariable timePassedBetweenEstimatorTicks = new LongYoVariable("timePassedBetweenEstimatorTicks", registry);
   
   
   private final BooleanYoVariable runController = new BooleanYoVariable("runController", registry);
   
   public DRCControllerThread(DRCRobotModel robotModel, MomentumBasedControllerFactory controllerFactory,
         ThreadDataSynchronizer threadDataSynchronizer, DRCOutputWriter outputWriter, GlobalDataProducer dataProducer, RobotVisualizer robotVisualizer,
         double gravity)
   {
      this.threadDataSynchronizer = threadDataSynchronizer;
      this.outputWriter = outputWriter;
      this.robotVisualizer = robotVisualizer;
      this.controlDTInNS = TimeTools.secondsToNanoSeconds(robotModel.getControllerDT());
      this.estimatorDTInNS = TimeTools.secondsToNanoSeconds(robotModel.getEstimatorDT());
      this.estimatorTicksPerControlTick = this.controlDTInNS / this.estimatorDTInNS;
      controllerFullRobotModel = threadDataSynchronizer.getControllerFullRobotModel();
      
      if (DRCConfigParameters.ALLOW_MODEL_CORRUPTION)
      {
         fullRobotModelCorruptor = new FullRobotModelCorruptor(controllerFullRobotModel, registry);
      }
      else
      {
         fullRobotModelCorruptor = null;
      }
      
      forceSensorDataHolderForController = threadDataSynchronizer.getControllerForceSensorDataHolder();

      outputWriter.setFullRobotModel(controllerFullRobotModel, threadDataSynchronizer.getControllerRawJointSensorDataHolderMap());
      outputWriter.setForceSensorDataHolderForController(forceSensorDataHolderForController);

      ArrayList<InverseDynamicsJoint> listOfJointsToIgnore = new ArrayList<>();
      
      DRCRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
      DRCRobotLidarParameters lidarParameters = sensorInformation.getLidarParameters(0);
      if(lidarParameters != null)
      {
         listOfJointsToIgnore.add(controllerFullRobotModel.getOneDoFJointByName(lidarParameters.getLidarSpindleJointName()));
      }

      String[] additionalJointsToIgnore = robotModel.getWalkingControllerParameters().getJointsToIgnoreInController();
      if (additionalJointsToIgnore != null)
      {
         for (String jointToIgnore : additionalJointsToIgnore)
         {
            listOfJointsToIgnore.add(controllerFullRobotModel.getOneDoFJointByName(jointToIgnore));
         }
      }

      controllerReferenceFrames = new ReferenceFrames(controllerFullRobotModel);

      robotController = createMomentumBasedController(controllerFullRobotModel, controllerReferenceFrames, sensorInformation,
            controllerFactory, controllerTime, robotModel.getControllerDT(), gravity, forceSensorDataHolderForController, yoGraphicsListRegistry,
            registry, dataProducer, listOfJointsToIgnore.toArray(new InverseDynamicsJoint[]{}));
      
      firstTick.set(true);
      registry.addChild(robotController.getYoVariableRegistry());
      registry.addChild(outputWriter.getControllerYoVariableRegistry());
      
      lastEstimatorStartTime.set(Long.MIN_VALUE);
      expectedEstimatorTick.set(estimatorDTInNS);
      
      if(robotVisualizer != null)
      {
         robotVisualizer.addRegistry(registry, yoGraphicsListRegistry);
      }
   }
   
   public FullRobotModelCorruptor getFullRobotModelCorruptor()
   {
      return fullRobotModelCorruptor;
   }

   public static RobotController createMomentumBasedController(SDFFullRobotModel controllerModel, ReferenceFrames referenceFramesForController,
         DRCRobotSensorInformation sensorInformation, MomentumBasedControllerFactory controllerFactory, DoubleYoVariable yoTime, double controlDT,
         double gravity, ForceSensorDataHolder forceSensorDataHolderForController, YoGraphicsListRegistry yoGraphicsListRegistry,
         YoVariableRegistry registry, GlobalDataProducer dataProducer, InverseDynamicsJoint... jointsToIgnore)
   {
      CenterOfMassJacobian centerOfMassJacobian = new CenterOfMassJacobian(controllerModel.getElevator());

      FullInverseDynamicsStructure inverseDynamicsStructureForController = createInverseDynamicsStructure(controllerModel);

      TwistCalculator twistCalculator = inverseDynamicsStructureForController.getTwistCalculator();

      if (CREATE_COM_CALIBRATION_TOOL)
      {
         try
         {
            CenterOfMassCalibrationTool centerOfMassCalibrationTool = new CenterOfMassCalibrationTool(controllerModel, forceSensorDataHolderForController, yoGraphicsListRegistry, registry);
            controllerFactory.addUpdatable(centerOfMassCalibrationTool);
         }
         catch(Exception e)
         {
            System.err.println("Couldn't create CenterOfMassCalibrationTool");
         }
      }
      
      if (SHOW_WRIST_SENSOR_WRENCHES && yoGraphicsListRegistry != null)
      {
          double forceVizScaling = 10.0;
          String includeOnlySensorsContainingThisName = "arm";
          ForceSensorDataVisualizer forceSensorDataVisualizer = new ForceSensorDataVisualizer(controllerModel, forceSensorDataHolderForController, includeOnlySensorsContainingThisName, forceVizScaling, yoGraphicsListRegistry, registry);
          controllerFactory.addUpdatable(forceSensorDataVisualizer);
      }
      
      
      RobotController robotController = controllerFactory.getController(controllerModel, referenceFramesForController, controlDT, gravity, yoTime,
            yoGraphicsListRegistry, twistCalculator, centerOfMassJacobian, forceSensorDataHolderForController,
            dataProducer, jointsToIgnore);
      final ModularSensorProcessor sensorProcessor = createSensorProcessor(twistCalculator, centerOfMassJacobian, referenceFramesForController);

      ModularRobotController modularRobotController = new ModularRobotController("DRCMomentumBasedController");
      modularRobotController.setSensorProcessor(sensorProcessor);
      modularRobotController.addRobotController(robotController);

      if (yoGraphicsListRegistry != null)
      {
        if (SHOW_INERTIA_GRAPHICS)
        {
            CommonInertiaElipsoidsVisualizer commonInertiaElipsoidsVisualizer = new CommonInertiaElipsoidsVisualizer(controllerModel.getElevator(),
                  yoGraphicsListRegistry);
            modularRobotController.addRobotController(commonInertiaElipsoidsVisualizer);
         }

        if (SHOW_REFERENCE_FRAMES)
        {
              InverseDynamicsMechanismReferenceFrameVisualizer inverseDynamicsMechanismReferenceFrameVisualizer = new InverseDynamicsMechanismReferenceFrameVisualizer(
                    controllerModel.getElevator(), yoGraphicsListRegistry, 0.5);
              modularRobotController.addRobotController(inverseDynamicsMechanismReferenceFrameVisualizer);
        }
        
        if (SHOW_JOINTAXIS_ZALIGN_FRAMES)
        {
              JointAxisVisualizer jointAxisVisualizer= new JointAxisVisualizer(controllerModel.getElevator(), yoGraphicsListRegistry, 0.3);
              modularRobotController.addRobotController(jointAxisVisualizer);
          
        }

        CommonHumanoidReferenceFramesVisualizer referenceFramesVisualizer = new CommonHumanoidReferenceFramesVisualizer(referenceFramesForController, yoGraphicsListRegistry);
        modularRobotController.addRobotController(referenceFramesVisualizer);
      }
      
      if (CREATE_DYNAMICALLY_CONSISTENT_NULLSPACE_EVALUATOR)
      {
         RobotController dynamicallyConsistentNullspaceEvaluator = new ConstrainedCenterOfMassJacobianEvaluator(controllerModel);
         modularRobotController.addRobotController(dynamicallyConsistentNullspaceEvaluator);
      }

      return modularRobotController;
   }

   public static FullInverseDynamicsStructure createInverseDynamicsStructure(FullRobotModel fullRobotModel)
   {
      RigidBody elevator = fullRobotModel.getElevator();
      SixDoFJoint rootInverseDynamicsJoint = fullRobotModel.getRootJoint();
      RigidBody estimationLink = fullRobotModel.getPelvis();

      FullInverseDynamicsStructure inverseDynamicsStructure = new FullInverseDynamicsStructure(elevator, estimationLink, rootInverseDynamicsJoint);

      return inverseDynamicsStructure;
   }

   private static ModularSensorProcessor createSensorProcessor(TwistCalculator twistCalculator, CenterOfMassJacobian centerOfMassJacobian,
         ReferenceFrames referenceFrames)
   {
      ModularSensorProcessor modularSensorProcessor = new ModularSensorProcessor("ModularSensorProcessor", "");
      modularSensorProcessor.addSensorProcessor(new ReferenceFrameUpdater(referenceFrames));
      modularSensorProcessor.addSensorProcessor(new TwistUpdater(twistCalculator));
      modularSensorProcessor.addSensorProcessor(new CenterOfMassJacobianUpdater(centerOfMassJacobian));
      modularSensorProcessor.addSensorProcessor(new CommonHumanoidReferenceFramesUpdater(referenceFrames));

      return modularSensorProcessor;
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void read(long currentClockTime)
   {
      runController.set(threadDataSynchronizer.receiveControllerState());
      
      
      
      if(runController.getBooleanValue())
      {
         // Skip the first estimatorTicksPerControlTick estimator ticks
         if(threadDataSynchronizer.getEstimatorTick() < estimatorTicksPerControlTick)
         {
            runController.set(false);
         }
         else
         {         
            long estimatorStartTime = threadDataSynchronizer.getEstimatorClockStartTime();
            long timestamp = threadDataSynchronizer.getTimestamp();
            controllerTime.set(TimeTools.nanoSecondstoSeconds(timestamp));
            actualControlDT.set(currentClockTime - controllerStartTime.getLongValue());
            
            if(expectedEstimatorTick.getLongValue() != threadDataSynchronizer.getEstimatorTick())
            {
               if(expectedEstimatorTick.getLongValue() > threadDataSynchronizer.getEstimatorTick())
               {
                  controllerLeadsEstimatorTicks.increment();
               }
               else if(expectedEstimatorTick.getLongValue() < threadDataSynchronizer.getEstimatorTick())
               {
                  controllerLagsEstimatorTicks.increment();
               }

               lastExpectedEstimatorTick.set(expectedEstimatorTick.getLongValue());
               lastEstimatorTick.set(threadDataSynchronizer.getEstimatorTick());
               lastEstimatorClockStartTime.set(threadDataSynchronizer.getEstimatorClockStartTime());
               lastControllerClockTime.set(currentClockTime);
               timePassedSinceEstimator.set(currentClockTime - lastEstimatorStartTime.getLongValue()) ; 
               timePassedBetweenEstimatorTicks.set(estimatorStartTime - lastEstimatorStartTime.getLongValue());
            }
            
            expectedEstimatorTick.set(threadDataSynchronizer.getEstimatorTick() + estimatorTicksPerControlTick);
            controllerStartTime.set(currentClockTime);
            lastEstimatorStartTime.set(estimatorStartTime);
         }
      }
      
   }

   @Override
   public void run()
   {
      if (runController.getBooleanValue())
      {
         if (firstTick.getBooleanValue())
         {
            robotController.initialize();
            outputWriter.initialize();
            firstTick.set(false);
         }
         controllerTimer.startMeasurement();
         robotController.doControl();
         controllerTimer.stopMeasurement();
      }
   }

   @Override
   public void write(long timestamp)
   {
      if (runController.getBooleanValue())
      {
         outputWriter.writeAfterController(TimeTools.secondsToNanoSeconds(controllerTime.getDoubleValue()));
         totalDelay.set(timestamp - lastEstimatorStartTime.getLongValue());         
         if(robotVisualizer != null)
         {
            robotVisualizer.update(TimeTools.secondsToNanoSeconds(controllerTime.getDoubleValue()), registry);
         }
      }      
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return getClass().getSimpleName();
   }

   @Override
   public YoGraphicsListRegistry getDynamicGraphicObjectsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

   @Override
   public long nextWakeupTime()
   {
      if (lastEstimatorStartTime.getLongValue() == Long.MIN_VALUE)
      {
         return Long.MIN_VALUE;
      }
      else
      {
         return lastEstimatorStartTime.getLongValue()  + controlDTInNS + estimatorDTInNS;
      }
   }
}
