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
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

public class FeedbackControllerTemplate
{
   private final Map<RigidBodyBasics, Integer> spatialFeedbackControllerTemplate = new HashMap<>();
   private final Map<RigidBodyBasics, Integer> orientationFeedbackControllerTemplate = new HashMap<>();
   private final Map<RigidBodyBasics, Integer> pointFeedbackControllerTemplate = new HashMap<>();
   private final Set<OneDoFJointBasics> oneDoFJointFeedbackControllerTemplate = new HashSet<>();
   private boolean enableCenterOfMassFeedbackController = false;

   public FeedbackControllerTemplate()
   {
   }

   public FeedbackControllerTemplate(FeedbackControlCommandList commandTemplate)
   {
      convert(commandTemplate);
   }

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

   public void enableSpatialFeedbackController(RigidBodyBasics endEffector)
   {
      enableSpatialFeedbackController(endEffector, 1);
   }

   public void enableSpatialFeedbackController(RigidBodyBasics endEffector, int numberOfControllers)
   {
      spatialFeedbackControllerTemplate.put(endEffector, numberOfControllers);
   }

   public void enableOrientationFeedbackController(RigidBodyBasics endEffector)
   {
      enableOrientationFeedbackController(endEffector, 1);
   }

   public void enableOrientationFeedbackController(RigidBodyBasics endEffector, int numberOfControllers)
   {
      orientationFeedbackControllerTemplate.put(endEffector, numberOfControllers);
   }

   public void enablePointFeedbackController(RigidBodyBasics endEffector)
   {
      enablePointFeedbackController(endEffector, 1);
   }

   public void enablePointFeedbackController(RigidBodyBasics endEffector, int numberOfControllers)
   {
      pointFeedbackControllerTemplate.put(endEffector, 1);
   }

   public void enableOneDoFJointFeedbackController(OneDoFJointBasics joint)
   {
      oneDoFJointFeedbackControllerTemplate.add(joint);
   }

   public void enableCenterOfMassFeedbackController()
   {
      enableCenterOfMassFeedbackController = true;
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
}
