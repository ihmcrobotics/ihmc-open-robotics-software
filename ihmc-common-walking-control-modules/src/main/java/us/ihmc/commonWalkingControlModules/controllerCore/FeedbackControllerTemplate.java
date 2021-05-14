package us.ihmc.commonWalkingControlModules.controllerCore;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OneDoFJointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerFactory;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

/**
 * A template should be used when creating a new {@code WholeBodyControllerCore}. It is used to
 * specify the collection of feedback controllers that will be needed at runtime.
 * <p>
 * The feedback controllers creates a set of {@code YoVariable}s which need to be instantiated
 * before starting a {@code YoVariableServer} to be accessible remotely.
 * </p>
 * <p>
 * By default, if a feedback controller is missing at runtime, an exception will be thrown. This
 * behavior can be changed to dynamically create any controller that is missing at runtime, but the
 * new controllers' {@code YoVariable}s cannot be guaranteed to be accessible from a visualizer and
 * the instantiation of a controller is not real-time safe.
 * </p>
 */
public class FeedbackControllerTemplate
{
   private boolean allowDynamicControllerConstruction = false;
   private final Map<RigidBodyBasics, Integer> spatialFeedbackControllerTemplate = new HashMap<>();
   private final Map<RigidBodyBasics, Integer> orientationFeedbackControllerTemplate = new HashMap<>();
   private final Map<RigidBodyBasics, Integer> pointFeedbackControllerTemplate = new HashMap<>();
   private final Set<OneDoFJointBasics> oneDoFJointFeedbackControllerTemplate = new HashSet<>();
   private boolean enableCenterOfMassFeedbackController = false;

   private FeedbackControllerFactory feedbackControllerFactory = new FeedbackControllerFactory();

   /**
    * Creates an empty template.
    */
   public FeedbackControllerTemplate()
   {
   }

   /**
    * Creates and configures a template for the given list of commands.
    * <p>
    * The command is unpacked and interpreted as follows:
    * <ul>
    * <li>For every end-effector found at least once in a {@code SpatialFeedbackControlCommand}, the
    * request for a single spatial feedback controller is added to the template.
    * <li>For every end-effector found at least once in a {@code PointFeedbackControlCommand}, the
    * request for a single point feedback controller is added to the template.
    * <li>For every end-effector found at least once in a {@code OrientationFeedbackControlCommand},
    * the request for a single orientation feedback controller is added to the template.
    * <li>For every joint found at least once in a {@code OneDoFJointFeedbackControlCommand}, the
    * request for a 1-DoF joint feedback controller is added to the template.
    * <li>If the command list contains at least one {@code CenterOfMassFeedbackControlCommand}, the
    * request for a single center of mass feedback controller is added.
    * </ul>
    * </p>
    * 
    * @param commandTemplate the list of commands to be converted into a template. Not modified.
    */
   public FeedbackControllerTemplate(FeedbackControlCommandList commandTemplate)
   {
      convert(commandTemplate);
   }

   /**
    * Configures a template for the given list of commands.
    * <p>
    * The command is unpacked and interpreted as follows:
    * <ul>
    * <li>For every end-effector found at least once in a {@code SpatialFeedbackControlCommand}, the
    * request for a single spatial feedback controller is added to the template.
    * <li>For every end-effector found at least once in a {@code PointFeedbackControlCommand}, the
    * request for a single point feedback controller is added to the template.
    * <li>For every end-effector found at least once in a {@code OrientationFeedbackControlCommand},
    * the request for a single orientation feedback controller is added to the template.
    * <li>For every joint found at least once in a {@code OneDoFJointFeedbackControlCommand}, the
    * request for a 1-DoF joint feedback controller is added to the template.
    * <li>If the command list contains at least one {@code CenterOfMassFeedbackControlCommand}, the
    * request for a single center of mass feedback controller is added.
    * </ul>
    * </p>
    * 
    * @param commandTemplate the list of commands to be converted into a template. Not modified.
    */
   public void convert(FeedbackControlCommandList commandTemplate)
   {
      for (int i = 0; i < commandTemplate.getNumberOfCommands(); i++)
      {
         FeedbackControlCommand<?> commandExample = commandTemplate.getCommand(i);
         ControllerCoreCommandType commandType = commandExample.getCommandType();
         switch (commandType)
         {
            case TASKSPACE:
               enableSpatialFeedbackController(((SpatialFeedbackControlCommand) commandExample).getEndEffector());
               break;
            case POINT:
               enablePointFeedbackController(((PointFeedbackControlCommand) commandExample).getEndEffector());
               break;
            case ORIENTATION:
               enableOrientationFeedbackController(((OrientationFeedbackControlCommand) commandExample).getEndEffector());
               break;
            case JOINTSPACE:
               enableOneDoFJointFeedbackController(((OneDoFJointFeedbackControlCommand) commandExample).getJoint());
               break;
            case MOMENTUM:
               enableCenterOfMassFeedbackController();
               break;
            case COMMAND_LIST:
               convert((FeedbackControlCommandList) commandExample);
               break;
            default:
               throw new RuntimeException("The command type: " + commandExample.getCommandType() + " is not handled.");
         }
      }
   }

   /**
    * Sets the restriction for dynamic allocation of feedback controllers.
    * <p>
    * <strong>WARNING</strong>: This feature must not be enabled when the controller core is used in a
    * real-time environment!
    * </p>
    * <p>
    * (Default) When disabled: at runtime if a feedback controller is missing, the
    * {@link WholeBodyFeedbackController} will throw a detailed exception which should result in the
    * end of a run.
    * </p>
    * <p>
    * When enabled: at runtime if a feedback controller is missing, the
    * {@link WholeBodyFeedbackController} will instantiate a new one and use it.
    * </p>
    * 
    * @param allowDynamicControllerConstruction {@code false} to prevent dynamic allocation (default,
    *                                           real-time safe), {@code true} to allow dynamic
    *                                           allocation (not real-time safe).
    */
   public void setAllowDynamicControllerConstruction(boolean allowDynamicControllerConstruction)
   {
      this.allowDynamicControllerConstruction = allowDynamicControllerConstruction;
   }

   /**
    * Requests a single spatial feedback controller for the given end-effector.
    * 
    * @param endEffector the rigid-body to be controlled by the feedback controller.
    */
   public void enableSpatialFeedbackController(RigidBodyBasics endEffector)
   {
      enableSpatialFeedbackController(endEffector, 1);
   }

   /**
    * Requests a number of spatial feedback controller(s) for the given end-effector.
    * 
    * @param endEffector         the rigid-body to be controlled by the feedback controller(s).
    * @param numberOfControllers the number of controllers to be created for the given end-effector.
    */
   public void enableSpatialFeedbackController(RigidBodyBasics endEffector, int numberOfControllers)
   {
      spatialFeedbackControllerTemplate.put(endEffector, numberOfControllers);
   }

   /**
    * Requests a single orientation feedback controller for the given end-effector.
    * 
    * @param endEffector the rigid-body to be controlled by the feedback controller.
    */
   public void enableOrientationFeedbackController(RigidBodyBasics endEffector)
   {
      enableOrientationFeedbackController(endEffector, 1);
   }

   /**
    * Requests a number of orientation feedback controller(s) for the given end-effector.
    * 
    * @param endEffector         the rigid-body to be controlled by the feedback controller(s).
    * @param numberOfControllers the number of controllers to be created for the given end-effector.
    */
   public void enableOrientationFeedbackController(RigidBodyBasics endEffector, int numberOfControllers)
   {
      orientationFeedbackControllerTemplate.put(endEffector, numberOfControllers);
   }

   /**
    * Requests a single point feedback controller for the given end-effector.
    * 
    * @param endEffector the rigid-body to be controlled by the feedback controller.
    */
   public void enablePointFeedbackController(RigidBodyBasics endEffector)
   {
      enablePointFeedbackController(endEffector, 1);
   }

   /**
    * Requests a number of point feedback controller(s) for the given end-effector.
    * 
    * @param endEffector         the rigid-body to be controlled by the feedback controller(s).
    * @param numberOfControllers the number of controllers to be created for the given end-effector.
    */
   public void enablePointFeedbackController(RigidBodyBasics endEffector, int numberOfControllers)
   {
      pointFeedbackControllerTemplate.put(endEffector, 1);
   }

   /**
    * Requests a single 1-DoF joint feedback controller for the given joint.
    * 
    * @param joint the joint to be controlled by the feedback controller.
    */
   public void enableOneDoFJointFeedbackController(OneDoFJointBasics joint)
   {
      oneDoFJointFeedbackControllerTemplate.add(joint);
   }

   /**
    * Requests a single feedback controller to control the center of mass.
    */
   public void enableCenterOfMassFeedbackController()
   {
      enableCenterOfMassFeedbackController = true;
   }

   /**
    * Sets the factory to use for creating the feedback controllers in
    * {@link WholeBodyFeedbackController}.
    * 
    * @param feedbackControllerFactory the new factory.
    */
   public void setFeedbackControllerFactory(FeedbackControllerFactory feedbackControllerFactory)
   {
      this.feedbackControllerFactory = feedbackControllerFactory;
   }

   public boolean isDynamicControllerConstructionAllowed()
   {
      return allowDynamicControllerConstruction;
   }

   public Map<RigidBodyBasics, Integer> getSpatialFeedbackControllerTemplate()
   {
      return spatialFeedbackControllerTemplate;
   }

   public Map<RigidBodyBasics, Integer> getOrientationFeedbackControllerTemplate()
   {
      return orientationFeedbackControllerTemplate;
   }

   public Map<RigidBodyBasics, Integer> getPointFeedbackControllerTemplate()
   {
      return pointFeedbackControllerTemplate;
   }

   public Set<OneDoFJointBasics> getOneDoFJointFeedbackControllerTemplate()
   {
      return oneDoFJointFeedbackControllerTemplate;
   }

   public boolean isCenterOfMassFeedbackControllerEnabled()
   {
      return enableCenterOfMassFeedbackController;
   }

   public FeedbackControllerFactory getFeedbackControllerFactory()
   {
      return feedbackControllerFactory;
   }
}
