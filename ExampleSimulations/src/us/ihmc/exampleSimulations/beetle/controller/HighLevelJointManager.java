package us.ihmc.exampleSimulations.beetle.controller;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.exampleSimulations.beetle.parameters.HexapodControllerParameters;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

public class HighLevelJointManager
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registy = new YoVariableRegistry(name);
   private final DoubleYoVariable desiredJointQ;
   private final RigidBody[] bodyToControl = new RigidBody[1];
   private final JointspaceFeedbackControlCommand jointspaceFeedbackControlCommand = new JointspaceFeedbackControlCommand();
   private final OneDoFJoint joint;
   
   public HighLevelJointManager(String prefix, OneDoFJoint joint, YoVariableRegistry parentRegistry)
   {
      this.joint = joint;
      desiredJointQ = new DoubleYoVariable(prefix + "_q_desired", registy);
      bodyToControl[0] = joint.getSuccessor();
      jointspaceFeedbackControlCommand.addJoint(joint, 0.0, 0.0, 0.0);
      jointspaceFeedbackControlCommand.setWeightForSolver(1.0);
      parentRegistry.addChild(registy);
   }

   public void doControl(HexapodControllerParameters parameters)
   {
      jointspaceFeedbackControlCommand.setGains(parameters.getJointGains(joint));
      jointspaceFeedbackControlCommand.setOneDoFJoint(0, desiredJointQ.getDoubleValue(), 0.0, 0.0);
   }
   
   public JointspaceFeedbackControlCommand getJointspaceFeedbackControlCommand()
   {
      return jointspaceFeedbackControlCommand;
   }

   public FeedbackControlCommand<?> getFeedbackControlTemplate()
   {
      return jointspaceFeedbackControlCommand;
   }

   public RigidBody[] getRigidBodiesToControl()
   {
      return bodyToControl;
   }
   
   public void initialize()
   {
      
   }
}
