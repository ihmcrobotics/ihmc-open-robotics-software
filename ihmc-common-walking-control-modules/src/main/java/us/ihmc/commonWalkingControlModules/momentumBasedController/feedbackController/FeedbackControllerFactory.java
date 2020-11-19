package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController;

import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.jointspace.OneDoFJointFeedbackController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace.CenterOfMassFeedbackController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace.OrientationFeedbackController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace.PointFeedbackController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace.SpatialFeedbackController;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

public class FeedbackControllerFactory
{
   protected WholeBodyControlCoreToolbox ccToolbox;
   protected FeedbackControllerToolbox fbToolbox;
   protected YoRegistry parentRegistry;

   public FeedbackControllerFactory()
   {
   }

   public void configure(WholeBodyControlCoreToolbox ccToolbox, FeedbackControllerToolbox fbToolbox, YoRegistry parentRegistry)
   {
      this.ccToolbox = ccToolbox;
      this.fbToolbox = fbToolbox;
      this.parentRegistry = parentRegistry;
   }

   public OneDoFJointFeedbackController buildOneDoFJointFeedbackController(OneDoFJointBasics joint)
   {
      return new OneDoFJointFeedbackController(joint, ccToolbox, fbToolbox, parentRegistry);
   }

   public OrientationFeedbackController buildOrientationFeedbackController(RigidBodyBasics endEffector, int controllerIndex)
   {
      return new OrientationFeedbackController(endEffector, controllerIndex, ccToolbox, fbToolbox, parentRegistry);
   }

   public PointFeedbackController buildPointFeedbackController(RigidBodyBasics endEffector, int controllerIndex)
   {
      return new PointFeedbackController(endEffector, controllerIndex, ccToolbox, fbToolbox, parentRegistry);
   }

   public SpatialFeedbackController buildSpatialFeedbackController(RigidBodyBasics endEffector, int controllerIndex)
   {
      return new SpatialFeedbackController(endEffector, controllerIndex, ccToolbox, fbToolbox, parentRegistry);
   }

   public CenterOfMassFeedbackController buildCenterOfMassFeedbackController()
   {
      return new CenterOfMassFeedbackController(ccToolbox, fbToolbox, parentRegistry);
   }
}
