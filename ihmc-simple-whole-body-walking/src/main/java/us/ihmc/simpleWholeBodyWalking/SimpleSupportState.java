package us.ihmc.simpleWholeBodyWalking;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.PIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPIDSE3Gains;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.weightMatrices.SolverWeightLevels;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * This is the active foot state when the foot is in flat support. Usually the command to the QP
 * should be a zero acceleration command. When the foot is barely loaded or the CoP gets close to
 * the edges of the foot polygon some of the directions start becoming feedback controlled. E.g.
 * when barely loaded x and y position as well as foot yaw are controlled to remain constant. The
 * state also contains the ability to shift the CoP around within the foothold in case the support
 * area needs to be explored.
 */
public class SimpleSupportState extends SimpleFootControlState
{
   private static final int dofs = Twist.SIZE;

   private final YoRegistry registry;

   private final FrameConvexPolygon2D footPolygon = new FrameConvexPolygon2D();

   private final FootSwitchInterface footSwitch;

   private final PoseReferenceFrame controlFrame;
   private final PoseReferenceFrame desiredSoleFrame;
   private final YoGraphicReferenceFrame frameViz;

   private final InverseDynamicsCommandList inverseDynamicsCommandsList = new InverseDynamicsCommandList();
   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();
   private final SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();

   private final SelectionMatrix6D accelerationSelectionMatrix = new SelectionMatrix6D();
   private final SelectionMatrix6D feedbackSelectionMatrix = new SelectionMatrix6D();

   private final FramePoint2D cop2d = new FramePoint2D();
   private final FramePoint3D framePosition = new FramePoint3D();
   private final FrameQuaternion frameOrientation = new FrameQuaternion();
   private final FramePose3D bodyFixedControlledPose = new FramePose3D();
   private final FramePoint3D desiredCopPosition = new FramePoint3D();

   private final FramePoint2D desiredCoP = new FramePoint2D();

   private final FramePoint3D footPosition = new FramePoint3D();
   private final FrameQuaternion footOrientation = new FrameQuaternion();

   // For line contact walking and balancing:
   private final BooleanProvider holdFootOrientationFlat;

   // For straight legs with privileged configuration
   private final RigidBodyBasics pelvis;

   private Vector3DReadOnly angularWeight;
   private Vector3DReadOnly linearWeight;

   private final PIDSE3GainsReadOnly gains;

   private final PIDSE3Gains localGains = new DefaultPIDSE3Gains();

   private final FrameVector3D fullyConstrainedNormalContactVector;


   public SimpleSupportState(ContactableFoot contactableFoot,
                             HighLevelHumanoidControllerToolbox controllerToolbox,
                             RobotSide robotSide,
                             FullHumanoidRobotModel fullRobotModel,
                             PIDSE3GainsReadOnly holdPositionGains, YoRegistry parentRegistry)
   {
      super(contactableFoot, controllerToolbox, robotSide, fullRobotModel);

      this.gains = holdPositionGains;

      String prefix = robotSide.getLowerCaseName() + "Foot";
      registry = new YoRegistry(prefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);

      fullyConstrainedNormalContactVector = new FrameVector3D(contactableFoot.getSoleFrame(), 0.0, 0.0, 1.0);

      footSwitch = controllerToolbox.getFootSwitches().get(robotSide);
      controlFrame = new PoseReferenceFrame(prefix + "HoldPositionFrame", contactableFoot.getSoleFrame());
      desiredSoleFrame = new PoseReferenceFrame(prefix + "DesiredSoleFrame", worldFrame);

      pelvis = fullRobotModel.getPelvis();

      spatialAccelerationCommand.setWeight(SolverWeightLevels.FOOT_SUPPORT_WEIGHT);
      spatialAccelerationCommand.set(rootBody, contactableFoot.getRigidBody());
      spatialAccelerationCommand.setPrimaryBase(pelvis);

      spatialFeedbackControlCommand.setWeightForSolver(SolverWeightLevels.FOOT_SUPPORT_WEIGHT);
      spatialFeedbackControlCommand.set(rootBody, contactableFoot.getRigidBody());
      spatialFeedbackControlCommand.setPrimaryBase(pelvis);

      desiredLinearVelocity.setToZero(worldFrame);
      desiredAngularVelocity.setToZero(worldFrame);
      desiredLinearAcceleration.setToZero(worldFrame);
      desiredAngularAcceleration.setToZero(worldFrame);


      holdFootOrientationFlat = new BooleanParameter(prefix + "HoldFlatOrientation", registry, false);

      YoGraphicsListRegistry graphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();
      if (graphicsListRegistry != null)
      {
         frameViz = new YoGraphicReferenceFrame(controlFrame, registry, false, 0.2);
         graphicsListRegistry.registerYoGraphic(prefix + getClass().getSimpleName(), frameViz);
      }
      else
      {
         frameViz = null;
      }
   }

   @Override
   public void onEntry()
   {
      super.onEntry();
      controllerToolbox.setFootContactStateNormalContactVector(robotSide, fullyConstrainedNormalContactVector);

      computeFootPolygon();
      updateHoldPositionSetpoints();
   }

   @Override
   public void onExit()
   {
      super.onExit();
      if (frameViz != null)
         frameViz.hide();
      desiredAngularVelocity.setToZero(worldFrame);
      desiredAngularAcceleration.setToZero(worldFrame);
   }

   @Override
   public void doSpecificAction(double timeInState)
   {
      computeFootPolygon();
      controllerToolbox.getDesiredCenterOfPressure(contactableFoot, desiredCoP);

      footSwitch.computeAndPackCoP(cop2d);
      if (cop2d.containsNaN())
         cop2d.setToZero(contactableFoot.getSoleFrame());

      YoPlaneContactState planeContactState = controllerToolbox.getFootContactState(robotSide);
      for (int i = 0; i < planeContactState.getTotalNumberOfContactPoints(); i++)
      {
         YoContactPoint contactPoint = planeContactState.getContactPoints().get(i);
         planeContactState.setMaxContactPointNormalForce(contactPoint, Double.POSITIVE_INFINITY);
      }

      updateHoldPositionSetpoints();

      localGains.set(gains);

      // update the control frame
      framePosition.setIncludingFrame(cop2d, 0.0);
      frameOrientation.setToZero(contactableFoot.getSoleFrame());
      controlFrame.setPoseAndUpdate(framePosition, frameOrientation);

      // assemble acceleration command
      ReferenceFrame bodyFixedFrame = contactableFoot.getRigidBody().getBodyFixedFrame();
      footAcceleration.setToZero(bodyFixedFrame, rootBody.getBodyFixedFrame(), controlFrame);
      footAcceleration.setBodyFrame(bodyFixedFrame);
      spatialAccelerationCommand.setSpatialAcceleration(controlFrame, footAcceleration);
      spatialAccelerationCommand.setWeights(angularWeight, linearWeight);

      // assemble feedback command
      bodyFixedControlledPose.setToZero(controlFrame);
      bodyFixedControlledPose.changeFrame(contactableFoot.getRigidBody().getBodyFixedFrame());
      desiredCopPosition.setIncludingFrame(cop2d, 0.0);
      desiredCopPosition.setReferenceFrame(desiredSoleFrame);
      desiredCopPosition.changeFrame(worldFrame);
      spatialFeedbackControlCommand.setControlFrameFixedInEndEffector(bodyFixedControlledPose);
      spatialFeedbackControlCommand.setInverseDynamics(desiredOrientation,
                                                       desiredCopPosition,
                                                       desiredAngularVelocity,
                                                       desiredLinearVelocity,
                                                       desiredAngularAcceleration,
                                                       desiredLinearAcceleration);
      spatialFeedbackControlCommand.setWeightsForSolver(angularWeight, linearWeight);
      spatialFeedbackControlCommand.setGains(localGains);

      // set selection matrices
      MovingReferenceFrame soleZUpFrame = controllerToolbox.getReferenceFrames().getSoleZUpFrame(robotSide);
      accelerationSelectionMatrix.resetSelection();
      accelerationSelectionMatrix.setSelectionFrame(soleZUpFrame);
      feedbackSelectionMatrix.resetSelection();
      feedbackSelectionMatrix.setSelectionFrame(soleZUpFrame);

      for (int i = dofs - 1; i >= 0; i--)
      {
         feedbackSelectionMatrix.selectAxis(i, false);
      }

      spatialAccelerationCommand.setSelectionMatrix(accelerationSelectionMatrix);
      spatialFeedbackControlCommand.setSelectionMatrix(feedbackSelectionMatrix);

      // update visualization
      if (frameViz != null)
         frameViz.setToReferenceFrame(controlFrame);
   }

   private void computeFootPolygon()
   {
      ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
      footPolygon.clear(soleFrame);
      for (int i = 0; i < contactableFoot.getTotalNumberOfContactPoints(); i++)
         footPolygon.addVertex(contactableFoot.getContactPoints2d().get(i));
      footPolygon.update();
   }

   private void updateHoldPositionSetpoints()
   {
      footPosition.setToZero(contactableFoot.getSoleFrame());
      footOrientation.setToZero(contactableFoot.getSoleFrame());
      MovingReferenceFrame soleZUpFrame = controllerToolbox.getReferenceFrames().getSoleZUpFrame(robotSide);
      footPosition.changeFrame(soleZUpFrame);
      footOrientation.changeFrame(soleZUpFrame);
      desiredPosition.changeFrame(soleZUpFrame);
      desiredOrientation.changeFrame(soleZUpFrame);

      // The z component is always updated as it is never held in place
      desiredPosition.set(footPosition);
      desiredOrientation.set(footOrientation);

      if (holdFootOrientationFlat.getValue())
         desiredOrientation.setYawPitchRoll(desiredOrientation.getYaw(), 0.0, 0.0);

      desiredPosition.changeFrame(worldFrame);
      desiredOrientation.changeFrame(worldFrame);
      desiredAngularVelocity.changeFrame(worldFrame);
      desiredAngularAcceleration.changeFrame(worldFrame);
      desiredSoleFrame.setPoseAndUpdate(desiredPosition, desiredOrientation);
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      inverseDynamicsCommandsList.clear();
      inverseDynamicsCommandsList.addCommand(spatialAccelerationCommand);

      return inverseDynamicsCommandsList;
   }

   @Override
   public SpatialFeedbackControlCommand getFeedbackControlCommand()
   {
      return spatialFeedbackControlCommand;
   }

   public void setWeights(Vector3DReadOnly angularWeight, Vector3DReadOnly linearWeight)
   {
      this.angularWeight = angularWeight;
      this.linearWeight = linearWeight;
   }
}
