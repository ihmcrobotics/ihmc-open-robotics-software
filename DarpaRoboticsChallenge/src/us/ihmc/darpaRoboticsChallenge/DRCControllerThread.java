package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.controllers.ControllerFactory;
import us.ihmc.commonWalkingControlModules.controllers.LidarControllerInterface;
import us.ihmc.commonWalkingControlModules.corruptors.FullRobotModelCorruptor;
import us.ihmc.commonWalkingControlModules.referenceFrames.ReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.CenterOfMassJacobianUpdater;
import us.ihmc.commonWalkingControlModules.sensors.CommonWalkingReferenceFramesUpdater;
import us.ihmc.commonWalkingControlModules.sensors.ReferenceFrameUpdater;
import us.ihmc.commonWalkingControlModules.sensors.TwistUpdater;
import us.ihmc.commonWalkingControlModules.visualizer.CommonInertiaElipsoidsVisualizer;
import us.ihmc.darpaRoboticsChallenge.controllers.ConstrainedCenterOfMassJacobianEvaluator;
import us.ihmc.darpaRoboticsChallenge.controllers.EstimationLinkHolder;
import us.ihmc.darpaRoboticsChallenge.controllers.concurrent.ThreadDataSynchronizer;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotContactPointParamaters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.darpaRoboticsChallenge.outputs.DRCOutputWriter;
import us.ihmc.sensorProcessing.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.io.streamingData.GlobalDataProducer;
import us.ihmc.utilities.math.TimeTools;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.InverseDynamicsMechanismReferenceFrameVisualizer;
import com.yobotics.simulationconstructionset.JointAxisVisualizer;
import com.yobotics.simulationconstructionset.LongYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.ModularRobotController;
import com.yobotics.simulationconstructionset.robotController.ModularSensorProcessor;
import com.yobotics.simulationconstructionset.robotController.MultiThreadedRobotControlElement;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.time.ExecutionTimer;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class DRCControllerThread implements MultiThreadedRobotControlElement
{
   private final YoVariableRegistry registry = new YoVariableRegistry("DRCControllerThread");
   private static final boolean CREATE_DYNAMICALLY_CONSISTENT_NULLSPACE_EVALUATOR = false;
   private static final boolean SHOW_INERTIA_GRAPHICS = false;
   private static final boolean SHOW_REFERENCE_FRAMES = false;
   private static final boolean SHOW_JOINTAXIS_ZALIGN_FRAMES = false;

   private final long controlDTInNS;
   
   private final DoubleYoVariable controllerTime = new DoubleYoVariable("controllerTime", registry);
   private final BooleanYoVariable firstTick = new BooleanYoVariable("firstTick", registry);
   

   private final SDFFullRobotModel controllerFullRobotModel;
   private final ReferenceFrames controllerReferenceFrames;

   private final DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
   private final ForceSensorDataHolder forceSensorDataHolderForController;

   private final ThreadDataSynchronizer threadDataSynchronizer;
   private final DRCOutputWriter outputWriter;
   
   private final RobotController robotController;
   
   private final ExecutionTimer controllerTimer = new ExecutionTimer("controllerTimer", 10.0, registry);
   private final LongYoVariable nextExecutionTime = new LongYoVariable("nextExecutionTime", registry);
   private final LongYoVariable totalDelay = new LongYoVariable("totalDelay", registry);
   
   private final BooleanYoVariable runController = new BooleanYoVariable("runController", registry);
   
   public DRCControllerThread(DRCRobotModel robotModel, ControllerFactory controllerFactory, LidarControllerInterface lidarControllerInterface,
         ThreadDataSynchronizer threadDataSynchronizer, DRCOutputWriter outputWriter, GlobalDataProducer dataProducer, double gravity)
   {
      this.threadDataSynchronizer = threadDataSynchronizer;
      this.outputWriter = outputWriter;
      this.controlDTInNS = TimeTools.secondsToNanoSeconds(robotModel.getControllerDT());
      controllerFullRobotModel = threadDataSynchronizer.getControllerFullRobotModel();
      
      new FullRobotModelCorruptor(controllerFullRobotModel, registry);
      
      
      forceSensorDataHolderForController = threadDataSynchronizer.getControllerForceSensorDataHolder();

      outputWriter.setFullRobotModel(controllerFullRobotModel);
      outputWriter.setForceSensorDataHolderForController(forceSensorDataHolderForController);
      
      DRCRobotJointMap jointMap = robotModel.getJointMap();
      DRCRobotPhysicalProperties physicalProperties = robotModel.getPhysicalProperties();
      DRCRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
      DRCRobotContactPointParamaters contactPointParamaters = robotModel.getContactPointParamaters(false, false);


      controllerReferenceFrames = new ReferenceFrames(controllerFullRobotModel, jointMap, physicalProperties.getAnkleHeight());
      controllerReferenceFrames.visualize(dynamicGraphicObjectsListRegistry, registry);

      robotController = createMomentumBasedController(controllerFullRobotModel, controllerReferenceFrames, sensorInformation,
            contactPointParamaters, controllerFactory, lidarControllerInterface, controllerTime, robotModel.getControllerDT(), gravity, forceSensorDataHolderForController,
            dynamicGraphicObjectsListRegistry, registry, dataProducer);
      
      firstTick.set(true);
      nextExecutionTime.set(0);
      registry.addChild(robotController.getYoVariableRegistry());
      registry.addChild(outputWriter.getControllerYoVariableRegistry());
      
      nextExecutionTime.set(Long.MIN_VALUE);
   }

   public static RobotController createMomentumBasedController(SDFFullRobotModel controllerModel, ReferenceFrames referenceFramesForController,
         DRCRobotSensorInformation sensorInformation, DRCRobotContactPointParamaters contactPointPatamaters, ControllerFactory controllerFactory,
         LidarControllerInterface lidarControllerInterface, DoubleYoVariable yoTime, double controlDT, double gravity,
         ForceSensorDataHolder forceSensorDataHolderForController, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
         YoVariableRegistry registry, GlobalDataProducer dataProducer)
   {
      CenterOfMassJacobian centerOfMassJacobian = new CenterOfMassJacobian(controllerModel.getElevator());

      FullInverseDynamicsStructure inverseDynamicsStructureForController = createInverseDynamicsStructure(controllerModel);

      OneDoFJoint lidarJoint = controllerModel.getOneDoFJointByName(sensorInformation.getLidarJointName());
      lidarControllerInterface.setLidarJoint(lidarJoint);

      TwistCalculator twistCalculator = inverseDynamicsStructureForController.getTwistCalculator();

      RobotController robotController = controllerFactory.getController(controllerModel, referenceFramesForController, controlDT, gravity, yoTime,
            dynamicGraphicObjectsListRegistry, twistCalculator, centerOfMassJacobian, forceSensorDataHolderForController,
            lidarControllerInterface, dataProducer, contactPointPatamaters.getFootGroundContactPointsInSoleFrameForController());
      final ModularSensorProcessor sensorProcessor = createSensorProcessor(twistCalculator, centerOfMassJacobian, referenceFramesForController);

      ModularRobotController modularRobotController = new ModularRobotController("DRCMomentumBasedController");
      modularRobotController.setSensorProcessor(sensorProcessor);
      modularRobotController.addRobotController(robotController);

      if (dynamicGraphicObjectsListRegistry != null)
      {
        if (SHOW_INERTIA_GRAPHICS)
        {
            CommonInertiaElipsoidsVisualizer commonInertiaElipsoidsVisualizer = new CommonInertiaElipsoidsVisualizer(controllerModel.getElevator(),
                  dynamicGraphicObjectsListRegistry);
            modularRobotController.addRobotController(commonInertiaElipsoidsVisualizer);
         }

        if (SHOW_REFERENCE_FRAMES)
        {
              InverseDynamicsMechanismReferenceFrameVisualizer inverseDynamicsMechanismReferenceFrameVisualizer = new InverseDynamicsMechanismReferenceFrameVisualizer(
                    controllerModel.getElevator(), dynamicGraphicObjectsListRegistry, 0.5);
              modularRobotController.addRobotController(inverseDynamicsMechanismReferenceFrameVisualizer);
        }
        
        if (SHOW_JOINTAXIS_ZALIGN_FRAMES)
        {
              JointAxisVisualizer jointAxisVisualizer= new JointAxisVisualizer(controllerModel.getElevator(), dynamicGraphicObjectsListRegistry, 0.3);
              modularRobotController.addRobotController(jointAxisVisualizer);
          
        }
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
      RigidBody estimationLink = EstimationLinkHolder.getEstimationLink(fullRobotModel); // rootInverseDynamicsJoint.getSuccessor();

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
      modularSensorProcessor.addSensorProcessor(new CommonWalkingReferenceFramesUpdater(referenceFrames));

      return modularSensorProcessor;
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void read(double time, long currentClockTime)
   {
      runController.set(threadDataSynchronizer.receiveControllerState());
      
      if(runController.getBooleanValue())
      {
         long estimatorStartTime = threadDataSynchronizer.getEstimatorClockStartTime();
         long timestamp = threadDataSynchronizer.getTimestamp();
         controllerTime.set(TimeTools.nanoSecondstoSeconds(timestamp));
         nextExecutionTime.set(estimatorStartTime + controlDTInNS);
      }
   }

   @Override
   public void run()
   {
      if(runController.getBooleanValue())
      {
      if(firstTick.getBooleanValue())
      {
         robotController.initialize();
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
      if(runController.getBooleanValue())
   {
      outputWriter.writeAfterController(TimeTools.secondsToNanoSeconds(controllerTime.getDoubleValue()));
         totalDelay.set(timestamp - nextExecutionTime.getLongValue() - controlDTInNS);
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
   public DynamicGraphicObjectsListRegistry getDynamicGraphicObjectsListRegistry()
   {
      return dynamicGraphicObjectsListRegistry;
   }

   @Override
   public long nextWakeupTime()
   {
      return nextExecutionTime.getLongValue();
   }
}
