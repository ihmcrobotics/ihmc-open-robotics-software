package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.ControllerFailureListener;
import us.ihmc.robotics.controllers.ControllerStateChangedListener;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.robotics.sensors.*;
import us.ihmc.sensorProcessing.model.RobotMotionStatusChangedListener;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.util.PeriodicThreadScheduler;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

public abstract class AbstractMomentumBasedControllerFactory implements CloseableAndDisposable
{
   /**
    * Specifies whether the inverse dynamics module of the {@link WholeBodyControllerCore} should be created or not.
    * <p>
    * This module is created by default as the {@link WalkingHighLevelHumanoidController} needs it.
    * </p>
    * 
    * @param setup whether to setup the inverse dynamics mode or not.
    */
   public abstract void setupControllerCoreInverseDynamicsMode(boolean setup);

   /**
    * Specifies whether the inverse kinematics module of the {@link WholeBodyControllerCore} should be created or not.
    * <p>
    * This module is not created by default to prevent creating unused {@link YoVariable}s.
    * </p>
    * 
    * @param setup whether to setup the inverse kinematics mode or not.
    */
   public abstract void setupControllerCoreInverseKinematicsMode(boolean setup);

   /**
    * Specifies whether the virtual model control module of the {@link WholeBodyControllerCore} should be created or not.
    * <p>
    * This module is not created by default to prevent creating unused {@link YoVariable}s.
    * </p>
    * 
    * @param setup whether to setup the virtual model control mode or not.
    */
   public abstract void setupControllerCoreVirtualModelControlMode(boolean setup);

   public abstract void setHeadingAndVelocityEvaluationScriptParameters(HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters);

   public abstract void addUpdatable(Updatable updatable);

   public abstract void setInverseDynamicsCalculatorListener(InverseDynamicsCalculatorListener inverseDynamicsCalculatorListener);

   public abstract void createControllerNetworkSubscriber(PeriodicThreadScheduler scheduler, PacketCommunicator packetCommunicator);

   public abstract void createComponentBasedFootstepDataMessageGenerator();

   public abstract void createComponentBasedFootstepDataMessageGenerator(boolean useHeadingAndVelocityScript);

   public abstract void createComponentBasedFootstepDataMessageGenerator(boolean useHeadingAndVelocityScript, HeightMap heightMapForFootstepZ);

   public abstract void createQueuedControllerCommandGenerator(ConcurrentLinkedQueue<Command<?, ?>> controllerCommands);

   public abstract void createUserDesiredControllerCommandGenerator();

   public abstract RobotController getController(FullHumanoidRobotModel fullRobotModel, double controlDT, double gravity, YoDouble yoTime,
                                        YoGraphicsListRegistry yoGraphicsListRegistry, ForceSensorDataHolderReadOnly forceSensorDataHolder,
                                        CenterOfMassDataHolderReadOnly centerOfMassDataHolder, ContactSensorHolder contactSensorHolder,
                                        CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator, InverseDynamicsJoint... jointsToIgnore);

   public abstract void reinitializeWalking(boolean keepPosition);

   public abstract void setListenToHighLevelStatePackets(boolean isListening);

   public abstract void attachControllerFailureListeners(List<ControllerFailureListener> listeners);

   public abstract void attachControllerFailureListener(ControllerFailureListener listener);

   public abstract void attachControllerStateChangedListeners(List<ControllerStateChangedListener> listeners);

   public abstract void attachControllerStateChangedListener(ControllerStateChangedListener listener);

   public abstract void attachRobotMotionStatusChangedListener(RobotMotionStatusChangedListener listener);

   public abstract CommandInputManager getCommandInputManager();

   public abstract StatusMessageOutputManager getStatusOutputManager();

   public abstract HighLevelHumanoidControllerToolbox getHighLevelHumanoidControllerToolbox();

   /**
    * Warms up the walking controller by running it a number of iterations
    * 
    * @param iterations
    */
   public abstract void warmupWalkingController(int iterations);

   public abstract HighLevelState getCurrentHighLevelState();
}
