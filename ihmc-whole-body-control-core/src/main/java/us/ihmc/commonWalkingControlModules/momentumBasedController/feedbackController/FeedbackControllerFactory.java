package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController;

import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerTemplate;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyFeedbackController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.jointspace.OneDoFJointFeedbackController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace.CenterOfMassFeedbackController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace.OrientationFeedbackController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace.PointFeedbackController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace.SpatialFeedbackController;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Factory used in {@link WholeBodyFeedbackController} to create the different feedback controllers
 * needed for a control session.
 * <p>
 * This class can be extended to override any of the default factories to create custom feedback
 * controllers. The custom factory should be passed to {@link FeedbackControllerTemplate} so the
 * controller core uses it.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class FeedbackControllerFactory
{
   protected WholeBodyControlCoreToolbox ccToolbox;
   protected FeedbackControllerToolbox fbToolbox;
   protected YoRegistry parentRegistry;

   public FeedbackControllerFactory()
   {
   }

   /**
    * Sets the different tools needed to create the feedback controllers.
    * 
    * @param ccToolbox      the general toolbox for the whole-body controller core.
    * @param fbToolbox      the feedback controller toolbox which is used to centralize the variables
    *                       used by the feedback controllers.
    * @param parentRegistry the registry to which each feedback controller's registry should be
    *                       attached.
    */
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
