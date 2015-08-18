package us.ihmc.wholeBodyController;

import java.util.ArrayList;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.corruptors.FullRobotModelCorruptor;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.MomentumBasedControllerFactory;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonHumanoidReferenceFramesVisualizer;
import us.ihmc.commonWalkingControlModules.sensors.CenterOfMassJacobianUpdater;
import us.ihmc.commonWalkingControlModules.sensors.ReferenceFrameUpdater;
import us.ihmc.commonWalkingControlModules.sensors.TwistUpdater;
import us.ihmc.commonWalkingControlModules.visualizer.CenterOfMassVisualizer;
import us.ihmc.commonWalkingControlModules.visualizer.CommonInertiaEllipsoidsVisualizer;
import us.ihmc.communication.packets.ControllerCrashNotificationPacket.CrashLocation;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.sensorProcessing.parameters.DRCRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.simulationconstructionset.InverseDynamicsMechanismReferenceFrameVisualizer;
import us.ihmc.simulationconstructionset.JointAxisVisualizer;
import us.ihmc.simulationconstructionset.robotController.ModularRobotController;
import us.ihmc.simulationconstructionset.robotController.ModularSensorProcessor;
import us.ihmc.simulationconstructionset.robotController.MultiThreadedRobotControlElement;
import us.ihmc.simulationconstructionset.robotController.OutputProcessor;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.humanoidRobotics.model.BaseFullRobotModel;
import us.ihmc.humanoidRobotics.model.RobotMotionStatus;
import us.ihmc.humanoidRobotics.model.RobotMotionStatusChangedListener;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotics.sensors.ContactSensorHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.humanoidRobotics.model.RobotMotionStatusHolder;
import us.ihmc.humanoidRobotics.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.tools.time.TimeTools;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.LongYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.humanoidRobot.visualizer.RobotVisualizer;
import us.ihmc.yoUtilities.time.ExecutionTimer;

public class DRCControllerThread implements MultiThreadedRobotControlElement
{
   private static final boolean CREATE_DYNAMICALLY_CONSISTENT_NULLSPACE_EVALUATOR = false;
   private static final boolean SHOW_INERTIA_GRAPHICS = false;
   private static final boolean SHOW_REFERENCE_FRAMES = false;
   private static final boolean SHOW_JOINTAXIS_ZALIGN_FRAMES = false;
   private static final boolean SHOW_LEG_COM = false;

   private static final boolean CREATE_COM_CALIBRATION_TOOL = false;
   private static final boolean ALLOW_MODEL_CORRUPTION = false;

   private final YoVariableRegistry registry = new YoVariableRegistry("DRCControllerThread");
   private final RobotVisualizer robotVisualizer;

   private final long controlDTInNS;
   private final long estimatorDTInNS;
   private final long estimatorTicksPerControlTick;

   private final DoubleYoVariable controllerTime = new DoubleYoVariable("controllerTime", registry);
   private final BooleanYoVariable firstTick = new BooleanYoVariable("firstTick", registry);

   private final SDFFullRobotModel controllerFullRobotModel;
   private final OutputProcessor outputProcessor;
   private final FullRobotModelCorruptor fullRobotModelCorruptor;

   private final HumanoidReferenceFrames controllerReferenceFrames;

   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private final ForceSensorDataHolderReadOnly forceSensorDataHolderForController;
   private final CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator;

   private final ThreadDataSynchronizerInterface threadDataSynchronizer;
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

   private final GlobalDataProducer globalDataProducer;
   
   private final RigidBodyTransform rootToWorldTransform = new RigidBodyTransform();
   private final ReferenceFrame rootFrame;

   public DRCControllerThread(WholeBodyControllerParameters robotModel, DRCRobotSensorInformation sensorInformation,
         MomentumBasedControllerFactory controllerFactory, ThreadDataSynchronizerInterface threadDataSynchronizer, DRCOutputWriter outputWriter,
         GlobalDataProducer dataProducer, RobotVisualizer robotVisualizer, double gravity, double estimatorDT)
   {
      this.threadDataSynchronizer = threadDataSynchronizer;
      this.outputWriter = outputWriter;
      this.robotVisualizer = robotVisualizer;
      this.controlDTInNS = TimeTools.secondsToNanoSeconds(robotModel.getControllerDT());
      this.estimatorDTInNS = TimeTools.secondsToNanoSeconds(estimatorDT);
      this.estimatorTicksPerControlTick = this.controlDTInNS / this.estimatorDTInNS;
      this.controllerFullRobotModel = threadDataSynchronizer.getControllerFullRobotModel();
      this.rootFrame = this.controllerFullRobotModel.getRootJoint().getFrameAfterJoint();
      this.outputProcessor = robotModel.getOutputProcessor(controllerFullRobotModel);
      this.globalDataProducer = dataProducer;

      if (ALLOW_MODEL_CORRUPTION)
      {
         fullRobotModelCorruptor = new FullRobotModelCorruptor(controllerFullRobotModel, registry);
      }
      else
      {
         fullRobotModelCorruptor = null;
      }

      forceSensorDataHolderForController = threadDataSynchronizer.getControllerForceSensorDataHolder();
      centerOfPressureDataHolderForEstimator = threadDataSynchronizer.getControllerCenterOfPressureDataHolder();

      outputWriter.setFullRobotModel(controllerFullRobotModel, threadDataSynchronizer.getControllerRawJointSensorDataHolderMap());
      outputWriter.setForceSensorDataHolderForController(forceSensorDataHolderForController);

      ArrayList<InverseDynamicsJoint> listOfJointsToIgnore = new ArrayList<>();

      DRCRobotLidarParameters lidarParameters = sensorInformation.getLidarParameters(0);
      if (lidarParameters != null)
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
      InverseDynamicsJoint[] arrayOfJointsToIgnore = listOfJointsToIgnore.toArray(new InverseDynamicsJoint[] {});

      controllerReferenceFrames = new HumanoidReferenceFrames(controllerFullRobotModel);

      robotController = createMomentumBasedController(controllerFullRobotModel, outputProcessor, controllerReferenceFrames, sensorInformation,
            controllerFactory, controllerTime, robotModel.getControllerDT(), gravity, forceSensorDataHolderForController, threadDataSynchronizer.getControllerContactSensorHolder(),
            centerOfPressureDataHolderForEstimator, yoGraphicsListRegistry, registry, dataProducer, arrayOfJointsToIgnore);

      createControllerRobotMotionStatusUpdater(controllerFactory, threadDataSynchronizer.getControllerRobotMotionStatusHolder());

      firstTick.set(true);
      registry.addChild(robotController.getYoVariableRegistry());
      registry.addChild(outputWriter.getControllerYoVariableRegistry());

      lastEstimatorStartTime.set(Long.MIN_VALUE);
      expectedEstimatorTick.set(estimatorDTInNS);

      if (robotVisualizer != null)
      {
         robotVisualizer.addRegistry(registry, yoGraphicsListRegistry);
      }
   }

   private void createControllerRobotMotionStatusUpdater(MomentumBasedControllerFactory controllerFactory,
         final RobotMotionStatusHolder controllerRobotMotionStatusHolder)
   {
      RobotMotionStatusChangedListener controllerRobotMotionStatusUpdater = new RobotMotionStatusChangedListener()
      {
         @Override
         public void robotMotionStatusHasChanged(RobotMotionStatus newStatus, double time)
         {
            controllerRobotMotionStatusHolder.setCurrentRobotMotionStatus(newStatus);
         }
      };

      controllerFactory.attachRobotMotionStatusChangedListener(controllerRobotMotionStatusUpdater);
   }

   public FullRobotModelCorruptor getFullRobotModelCorruptor()
   {
      return fullRobotModelCorruptor;
   }

   public static RobotController createMomentumBasedController(SDFFullRobotModel controllerModel, OutputProcessor outputProcessor,
         HumanoidReferenceFrames referenceFramesForController, DRCRobotSensorInformation sensorInformation, MomentumBasedControllerFactory controllerFactory,
         DoubleYoVariable yoTime, double controlDT, double gravity, ForceSensorDataHolderReadOnly forceSensorDataHolderForController, ContactSensorHolder contactSensorHolder,
         CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry registry,
         GlobalDataProducer dataProducer, InverseDynamicsJoint... jointsToIgnore)
   {
      CenterOfMassJacobian centerOfMassJacobian = new CenterOfMassJacobian(controllerModel.getElevator());

      FullInverseDynamicsStructure inverseDynamicsStructureForController = createInverseDynamicsStructure(controllerModel);

      TwistCalculator twistCalculator = inverseDynamicsStructureForController.getTwistCalculator();

      if (CREATE_COM_CALIBRATION_TOOL)
      {
         try
         {
            CenterOfMassCalibrationTool centerOfMassCalibrationTool = new CenterOfMassCalibrationTool(controllerModel, forceSensorDataHolderForController,
                  yoGraphicsListRegistry, registry);
            controllerFactory.addUpdatable(centerOfMassCalibrationTool);
         }
         catch (Exception e)
         {
            System.err.println("Couldn't create CenterOfMassCalibrationTool");
         }
      }

      if (SHOW_LEG_COM)
      {
         for (RobotSide side : RobotSide.values)
         {
            String name = side.name() + "Leg";
            CenterOfMassVisualizer comVisualizer = new CenterOfMassVisualizer(name, controllerModel.getLegJoint(side, LegJointName.HIP_PITCH), twistCalculator,
                  registry, yoGraphicsListRegistry);
            controllerFactory.addUpdatable(comVisualizer);
         }
      }

      RobotController robotController = controllerFactory.getController(controllerModel, referenceFramesForController, controlDT, gravity, yoTime,
            yoGraphicsListRegistry, twistCalculator, centerOfMassJacobian, forceSensorDataHolderForController, contactSensorHolder, centerOfPressureDataHolderForEstimator,
            dataProducer, jointsToIgnore);
      final ModularSensorProcessor sensorProcessor = createSensorProcessor(twistCalculator, centerOfMassJacobian, referenceFramesForController);

      ModularRobotController modularRobotController = new ModularRobotController("DRCMomentumBasedController");
      modularRobotController.setSensorProcessor(sensorProcessor);
      modularRobotController.addRobotController(robotController);
      if (outputProcessor != null)
         modularRobotController.setOutputProcessor(outputProcessor);

      if (yoGraphicsListRegistry != null)
      {
         if (SHOW_INERTIA_GRAPHICS)
         {
            CommonInertiaEllipsoidsVisualizer commonInertiaElipsoidsVisualizer = new CommonInertiaEllipsoidsVisualizer(controllerModel.getElevator(),
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
            JointAxisVisualizer jointAxisVisualizer = new JointAxisVisualizer(controllerModel.getElevator(), yoGraphicsListRegistry, 0.3);
            modularRobotController.addRobotController(jointAxisVisualizer);
         }

         CommonHumanoidReferenceFramesVisualizer referenceFramesVisualizer = new CommonHumanoidReferenceFramesVisualizer(referenceFramesForController,
               yoGraphicsListRegistry);
         modularRobotController.addRobotController(referenceFramesVisualizer);
      }

      if (CREATE_DYNAMICALLY_CONSISTENT_NULLSPACE_EVALUATOR)
      {
         RobotController dynamicallyConsistentNullspaceEvaluator = new ConstrainedCenterOfMassJacobianEvaluator(controllerModel);
         modularRobotController.addRobotController(dynamicallyConsistentNullspaceEvaluator);
      }

      return modularRobotController;
   }

   public static FullInverseDynamicsStructure createInverseDynamicsStructure(BaseFullRobotModel fullRobotModel)
   {
      RigidBody elevator = fullRobotModel.getElevator();
      SixDoFJoint rootInverseDynamicsJoint = fullRobotModel.getRootJoint();
      RigidBody estimationLink = fullRobotModel.getPelvis();

      FullInverseDynamicsStructure inverseDynamicsStructure = new FullInverseDynamicsStructure(elevator, estimationLink, rootInverseDynamicsJoint);

      return inverseDynamicsStructure;
   }

   private static ModularSensorProcessor createSensorProcessor(TwistCalculator twistCalculator, CenterOfMassJacobian centerOfMassJacobian,
         HumanoidReferenceFrames referenceFrames)
   {
      ModularSensorProcessor modularSensorProcessor = new ModularSensorProcessor("ModularSensorProcessor", "");
      modularSensorProcessor.addSensorProcessor(new ReferenceFrameUpdater(referenceFrames));
      modularSensorProcessor.addSensorProcessor(new TwistUpdater(twistCalculator));
      modularSensorProcessor.addSensorProcessor(new CenterOfMassJacobianUpdater(centerOfMassJacobian));

      return modularSensorProcessor;
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void read(long currentClockTime)
   {
      try
      {
         runController.set(threadDataSynchronizer.receiveEstimatorStateForController());

         if (runController.getBooleanValue())
         {
            // Skip the first estimatorTicksPerControlTick estimator ticks
            if (threadDataSynchronizer.getEstimatorTick() < estimatorTicksPerControlTick)
            {
               runController.set(false);
            }
            else
            {
               long estimatorStartTime = threadDataSynchronizer.getEstimatorClockStartTime();
               long timestamp = threadDataSynchronizer.getTimestamp();
               controllerTime.set(TimeTools.nanoSecondstoSeconds(timestamp));
               actualControlDT.set(currentClockTime - controllerStartTime.getLongValue());

               if (expectedEstimatorTick.getLongValue() != threadDataSynchronizer.getEstimatorTick())
               {
                  if (expectedEstimatorTick.getLongValue() > threadDataSynchronizer.getEstimatorTick())
                  {
                     controllerLeadsEstimatorTicks.increment();
                  }
                  else if (expectedEstimatorTick.getLongValue() < threadDataSynchronizer.getEstimatorTick())
                  {
                     controllerLagsEstimatorTicks.increment();
                  }

                  lastExpectedEstimatorTick.set(expectedEstimatorTick.getLongValue());
                  lastEstimatorTick.set(threadDataSynchronizer.getEstimatorTick());
                  lastEstimatorClockStartTime.set(threadDataSynchronizer.getEstimatorClockStartTime());
                  lastControllerClockTime.set(currentClockTime);
                  timePassedSinceEstimator.set(currentClockTime - lastEstimatorStartTime.getLongValue());
                  timePassedBetweenEstimatorTicks.set(estimatorStartTime - lastEstimatorStartTime.getLongValue());
               }

               expectedEstimatorTick.set(threadDataSynchronizer.getEstimatorTick() + estimatorTicksPerControlTick);
               controllerStartTime.set(currentClockTime);
               lastEstimatorStartTime.set(estimatorStartTime);
            }
         }
      }
      catch (Exception e)
      {
         if(globalDataProducer != null)
         {
            globalDataProducer.notifyControllerCrash(CrashLocation.CONTROLLER_READ, e.getMessage());
         }
         
         throw new RuntimeException(e);
      }
   }

   @Override
   public void run()
   {
      try
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
      catch (Exception e)
      {
         if(globalDataProducer != null)
            globalDataProducer.notifyControllerCrash(CrashLocation.CONTROLLER_RUN, e.getMessage());
         throw new RuntimeException(e);
      }
   }

   @Override
   public void write(long timestamp)
   {
      try
      {
         if (runController.getBooleanValue())
         {
            outputWriter.writeAfterController(TimeTools.secondsToNanoSeconds(controllerTime.getDoubleValue()));
            totalDelay.set(timestamp - lastEstimatorStartTime.getLongValue());

            threadDataSynchronizer.publishControllerData();
            if (robotVisualizer != null)
            {
               robotVisualizer.update(TimeTools.secondsToNanoSeconds(controllerTime.getDoubleValue()), registry);
            }
            
            rootFrame.getTransformToDesiredFrame(rootToWorldTransform, ReferenceFrame.getWorldFrame());
            yoGraphicsListRegistry.setControllerTransformToWorld(rootToWorldTransform);
         }

      }
      catch (Exception e)
      {
         globalDataProducer.notifyControllerCrash(CrashLocation.CONTROLLER_WRITE, e.getMessage());
         throw new RuntimeException(e);

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
         return lastEstimatorStartTime.getLongValue() + controlDTInNS + estimatorDTInNS;
      }
   }

   public RobotController getRobotController()
   {
      return robotController;
   }

   public BaseFullRobotModel getFullRobotModel()
   {
      return controllerFullRobotModel;
   }
}
