package us.ihmc.darpaRoboticsChallenge;

import java.util.Arrays;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.InverseDynamicsMechanismReferenceFrameVisualizer;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.gui.GUISetterUpperRegistry;
import com.yobotics.simulationconstructionset.robotController.ModularRobotController;
import com.yobotics.simulationconstructionset.robotController.ModularSensorProcessor;
import com.yobotics.simulationconstructionset.robotController.MultiThreadedRobotControlElement;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.controllers.ControllerFactory;
import us.ihmc.commonWalkingControlModules.controllers.LidarControllerInterface;
import us.ihmc.commonWalkingControlModules.referenceFrames.ReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.CenterOfMassJacobianUpdater;
import us.ihmc.commonWalkingControlModules.sensors.CommonWalkingReferenceFramesUpdater;
import us.ihmc.commonWalkingControlModules.sensors.ReferenceFrameUpdater;
import us.ihmc.commonWalkingControlModules.sensors.TwistUpdater;
import us.ihmc.commonWalkingControlModules.visualizer.CommonInertiaElipsoidsVisualizer;
import us.ihmc.darpaRoboticsChallenge.controllers.ConstrainedCenterOfMassJacobianEvaluator;
import us.ihmc.darpaRoboticsChallenge.controllers.EstimationLinkHolder;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCContactPointInformationFactory;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotContactPointParamaters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.io.streamingData.GlobalDataProducer;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.TwistCalculator;

public class DRCControllerThread implements MultiThreadedRobotControlElement
{
   private final YoVariableRegistry registry = new YoVariableRegistry("DRCControllerThread");
   private static final boolean CREATE_DYNAMICALLY_CONSISTENT_NULLSPACE_EVALUATOR = false;
   private static final boolean SHOW_INERTIA_GRAPHICS = false;
   private static final boolean SHOW_REFERENCE_FRAMES = false;

   private final DoubleYoVariable t = new DoubleYoVariable("t", registry);

   private final SDFFullRobotModel controllerFullRobotModel;
   private final ReferenceFrames controllerReferenceFrames;

   private final DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
   private final ForceSensorDataHolder forceSensorDataHolderForController;

   public DRCControllerThread(DRCRobotModel robotModel, ControllerFactory controllerFactory, LidarControllerInterface lidarControllerInterface,
         GlobalDataProducer dataProducer, double controlDT, double gravity)
   {
      controllerFullRobotModel = robotModel.createFullRobotModel();
      DRCRobotJointMap jointMap = robotModel.getJointMap();
      DRCRobotPhysicalProperties physicalProperties = robotModel.getPhysicalProperties();
      DRCRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
      DRCRobotContactPointParamaters contactPointParamaters = robotModel.getContactPointParamaters(false, false);

      forceSensorDataHolderForController = new ForceSensorDataHolder(Arrays.asList(controllerFullRobotModel.getForceSensorDefinitions()),
            controllerFullRobotModel.getRootJoint());

      controllerReferenceFrames = new ReferenceFrames(controllerFullRobotModel, jointMap, physicalProperties.getAnkleHeight());
      controllerReferenceFrames.visualize(dynamicGraphicObjectsListRegistry, registry);

      RobotController robotController = createMomentumBasedController(controllerFullRobotModel, controllerReferenceFrames, sensorInformation,
            contactPointParamaters, controllerFactory, lidarControllerInterface, t, controlDT, gravity, forceSensorDataHolderForController,
            dynamicGraphicObjectsListRegistry, registry, dataProducer);

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

      if (SHOW_INERTIA_GRAPHICS)
      {
         if (dynamicGraphicObjectsListRegistry != null)
         {
            CommonInertiaElipsoidsVisualizer commonInertiaElipsoidsVisualizer = new CommonInertiaElipsoidsVisualizer(controllerModel.getElevator(),
                  dynamicGraphicObjectsListRegistry);
            modularRobotController.addRobotController(commonInertiaElipsoidsVisualizer);
         }
      }

      if (SHOW_REFERENCE_FRAMES)
      {
         if (dynamicGraphicObjectsListRegistry != null)
         {
            InverseDynamicsMechanismReferenceFrameVisualizer inverseDynamicsMechanismReferenceFrameVisualizer = new InverseDynamicsMechanismReferenceFrameVisualizer(
                  controllerModel.getElevator(), dynamicGraphicObjectsListRegistry, 0.5);
            modularRobotController.addRobotController(inverseDynamicsMechanismReferenceFrameVisualizer);
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
      // TODO Auto-generated method stub

   }

   @Override
   public void read(double time)
   {
      t.set(time);
   }

   @Override
   public void run()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void write()
   {
      // TODO Auto-generated method stub

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
}
