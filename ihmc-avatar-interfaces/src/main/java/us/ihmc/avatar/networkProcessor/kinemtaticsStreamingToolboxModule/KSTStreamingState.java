package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage;
import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController.OutputPublisher;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.YoPIDSE3Gains;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class KSTStreamingState implements State
{
   private final KSTTools tools;
   private OutputPublisher outputPublisher = m ->
   {
   };
   private final YoDouble timeSinceLastMessageToController;
   private final YoDouble publishingPeriod;
   private final KinematicsToolboxConfigurationMessage configurationMessage = new KinematicsToolboxConfigurationMessage();
   private final FullHumanoidRobotModel desiredFullRobotModel;
   private final CommandInputManager ikCommandInputManager;

   private final KinematicsToolboxRigidBodyMessage pelvisOrientationMessage = new KinematicsToolboxRigidBodyMessage();
   private final RigidBodyBasics pelvis;

   private YoBoolean isStreaming;

   private YoDouble defaultLinearWeight, defaultAngularWeight;

   public KSTStreamingState(KSTTools tools)
   {
      this.tools = tools;
      configurationMessage.setJointVelocityWeight(10.0);
      desiredFullRobotModel = tools.getDesiredFullRobotModel();
      ikCommandInputManager = tools.getIKCommandInputManager();

      pelvis = desiredFullRobotModel.getPelvis();
      pelvisOrientationMessage.setEndEffectorHashCode(pelvis.hashCode());
      pelvisOrientationMessage.getDesiredOrientationInWorld().setToZero();
      pelvisOrientationMessage.getLinearSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(false, false, false));
      pelvisOrientationMessage.getAngularSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(true, true, false, ReferenceFrame.getWorldFrame()));
      pelvisOrientationMessage.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(1.0));

      YoVariableRegistry registry = tools.getRegistry();
      defaultLinearWeight = new YoDouble("defaultLinearWeight", registry);
      defaultAngularWeight = new YoDouble("defaultAngularWeight", registry);
      defaultLinearWeight.set(20.0);
      defaultAngularWeight.set(1.0);

      timeSinceLastMessageToController = new YoDouble("timeSinceLastMessageToController", registry);
      publishingPeriod = new YoDouble("publishingPeriod", registry);
      publishingPeriod.set(2.0 * tools.getWalkingControllerPeriod());
      isStreaming = new YoBoolean("isStreaming", registry);
   }

   public void setOutputPublisher(OutputPublisher outputPublisher)
   {
      this.outputPublisher = outputPublisher;
   }

   @Override
   public void onEntry()
   {
      timeSinceLastMessageToController.set(Double.POSITIVE_INFINITY);
      YoPIDSE3Gains defaultGains = tools.getIKController().getDefaultGains();
      defaultGains.setPositionProportionalGains(50.0);
      defaultGains.setPositionMaxFeedbackAndFeedbackRate(2.0, Double.POSITIVE_INFINITY);
      defaultGains.setOrientationProportionalGains(50.0);
      defaultGains.setOrientationMaxFeedbackAndFeedbackRate(35.0, Double.POSITIVE_INFINITY);
      configurationMessage.setJointVelocityWeight(1.0);
      ikCommandInputManager.submitMessage(configurationMessage);
   }

   @Override
   public void doAction(double timeInState)
   {
      KinematicsStreamingToolboxInputCommand latestInput = tools.pollInputCommand();

      if (latestInput != null)
      {
         for (int i = 0; i < latestInput.getNumberOfInputs(); i++)
         {
            KinematicsToolboxRigidBodyCommand input = latestInput.getInput(i);
            setDefaultWeightIfNeeded(input.getSelectionMatrix(), input.getWeightMatrix());
            ikCommandInputManager.submitCommand(input);
         }

         if (!latestInput.hasInputFor(pelvis))
            ikCommandInputManager.submitMessage(pelvisOrientationMessage);

         isStreaming.set(latestInput.getStreamToController());
      }

      tools.getIKController().updateInternal();

      if (isStreaming.getValue())
      {
         timeSinceLastMessageToController.add(tools.getToolboxControllerPeriod());

         if (timeSinceLastMessageToController.getValue() >= publishingPeriod.getValue())
         {
            outputPublisher.publish(tools.convertIKOutput());
            timeSinceLastMessageToController.set(0.0);
         }
      }
      else
      {
         timeSinceLastMessageToController.set(Double.POSITIVE_INFINITY);
      }
   }

   private void setDefaultWeightIfNeeded(SelectionMatrix6D selectionMatrix, WeightMatrix6D weightMatrix)
   {
      setDefaultWeightIfNeeded(selectionMatrix.getLinearPart(), weightMatrix.getLinearPart(), defaultLinearWeight.getValue());
      setDefaultWeightIfNeeded(selectionMatrix.getAngularPart(), weightMatrix.getAngularPart(), defaultAngularWeight.getValue());
   }
   
   private void setDefaultWeightIfNeeded(SelectionMatrix3D selectionMatrix, WeightMatrix3D weightMatrix, double defaultWeight)
   {
      if (selectionMatrix.isXSelected())
      {
         if (Double.isNaN(weightMatrix.getX()) || weightMatrix.getX() <= 0.0)
            weightMatrix.setXAxisWeight(defaultWeight);
      }
      if (selectionMatrix.isYSelected())
      {
         if (Double.isNaN(weightMatrix.getY()) || weightMatrix.getY() <= 0.0)
            weightMatrix.setYAxisWeight(defaultWeight);
      }
      if (selectionMatrix.isZSelected())
      {
         if (Double.isNaN(weightMatrix.getZ()) || weightMatrix.getZ() <= 0.0)
            weightMatrix.setZAxisWeight(defaultWeight);
      }
   }

   @Override
   public void onExit()
   {
   }
}
