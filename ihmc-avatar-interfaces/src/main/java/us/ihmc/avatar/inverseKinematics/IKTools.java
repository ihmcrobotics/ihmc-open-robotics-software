package us.ihmc.avatar.inverseKinematics;

import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxOptimizationSettings;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerTemplate;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointTorqueSoftLimitWeightCalculator;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Objects;

public class IKTools
{
   /** The gains of the solver depend on this dt, it shouldn't change based on the actual thread scheduled period. */
   public static final double CONTROL_DT = 0.001;
   public static final double GRAVITY = 9.81;

   public static WholeBodyControllerCore createControllerCore(FloatingJointBasics rootJoint,
                                                              RigidBodyBasics rootBody,
                                                              OneDoFJointBasics[] oneDoFJoints,
                                                              Collection<? extends RigidBodyBasics> controllableRigidBodies,
                                                              ReferenceFrame centerOfMassFrame,
                                                              KinematicsToolboxOptimizationSettings optimizationSettings,
                                                              YoRegistry registry)
   {
      // For IK, the root joint is a controlled joint.
      JointBasics[] controlledJoints = new JointBasics[oneDoFJoints.length + 1];
      controlledJoints[0] = rootJoint;
      System.arraycopy(oneDoFJoints, 0, controlledJoints, 1, oneDoFJoints.length);
      YoGraphicsListRegistry yoGraphicsListRegistry = null; // opt out
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(CONTROL_DT,
                                                                            GRAVITY,
                                                                            rootJoint,
                                                                            controlledJoints,
                                                                            centerOfMassFrame,
                                                                            optimizationSettings,
                                                                            yoGraphicsListRegistry,
                                                                            registry);
      toolbox.setJointPrivilegedConfigurationParameters(new JointPrivilegedConfigurationParameters());
      JointTorqueSoftLimitWeightCalculator jointTorqueMinimizationWeightCalculator = new JointTorqueSoftLimitWeightCalculator(toolbox.getJointIndexHandler());
      jointTorqueMinimizationWeightCalculator.setParameters(0.0, 0.001, 0.10);
      toolbox.setupForInverseKinematicsSolver(jointTorqueMinimizationWeightCalculator);
      FeedbackControllerTemplate controllerCoreTemplate = createFeedbackControllerTemplate(rootBody, controllableRigidBodies);
      JointDesiredOutputList lowLevelControllerOutput = new JointDesiredOutputList(oneDoFJoints);
      return new WholeBodyControllerCore(toolbox, controllerCoreTemplate, lowLevelControllerOutput, registry);
   }

   private static FeedbackControllerTemplate createFeedbackControllerTemplate(RigidBodyBasics rootBody,
                                                                              Collection<? extends RigidBodyBasics> controllableRigidBodies)
   {
      FeedbackControllerTemplate template = new FeedbackControllerTemplate();
      template.setAllowDynamicControllerConstruction(true);
      template.enableCenterOfMassFeedbackController();
      Collection<? extends RigidBodyBasics> rigidBodies;

      if (controllableRigidBodies != null)
         rigidBodies = controllableRigidBodies;
      else
         rigidBodies = rootBody.subtreeList();

      int numberOfControllersPerBody = 1;
      rigidBodies.stream().forEach(rigidBody -> template.enableSpatialFeedbackController(rigidBody, numberOfControllersPerBody));

      SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).forEach(template::enableOneDoFJointFeedbackController);
      return template;
   }
}
