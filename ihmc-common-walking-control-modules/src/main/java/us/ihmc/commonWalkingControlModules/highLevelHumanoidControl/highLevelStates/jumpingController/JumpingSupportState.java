package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.weightMatrices.SolverWeightLevels;
import us.ihmc.yoVariables.registry.YoRegistry;

public class JumpingSupportState implements JumpingFootControlState
{
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   protected final JumpingFootControlHelper footControlHelper;

   private final RobotSide robotSide;
   private final RigidBodyBasics rootBody;
   private final RigidBodyBasics pelvis;
   private final ContactableFoot contactableFoot;

   private final SpatialAcceleration footAcceleration = new SpatialAcceleration();

   private final JumpingControllerToolbox controllerToolbox;

   private final YoRegistry registry;

   private final FrameConvexPolygon2D footPolygon = new FrameConvexPolygon2D();

   private final FootSwitchInterface footSwitch;

   private final PoseReferenceFrame controlFrame;
   private final YoGraphicReferenceFrame frameViz;

   private Vector3DReadOnly angularWeight;
   private Vector3DReadOnly linearWeight;

   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();

   private final SelectionMatrix6D accelerationSelectionMatrix = new SelectionMatrix6D();

   private final FramePoint2D cop2d = new FramePoint2D();
   private final FramePoint3D framePosition = new FramePoint3D();
   private final FrameQuaternion frameOrientation = new FrameQuaternion();

   public JumpingSupportState(JumpingFootControlHelper footControlHelper, YoRegistry parentRegistry)
   {
      this.footControlHelper = footControlHelper;
      contactableFoot = footControlHelper.getContactableFoot();

      controllerToolbox = footControlHelper.getHighLevelHumanoidControllerToolbox();

      robotSide = footControlHelper.getRobotSide();
      FullHumanoidRobotModel fullRobotModel = footControlHelper.getHighLevelHumanoidControllerToolbox().getFullRobotModel();
      pelvis = fullRobotModel.getPelvis();
      rootBody = fullRobotModel.getElevator();

      String prefix = footControlHelper.getRobotSide().getLowerCaseName() + "Foot";
      registry = new YoRegistry(prefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);

      footSwitch = footControlHelper.getHighLevelHumanoidControllerToolbox().getFootSwitches().get(robotSide);
      controlFrame = new PoseReferenceFrame(prefix + "HoldPositionFrame", contactableFoot.getSoleFrame());

      spatialAccelerationCommand.setWeight(SolverWeightLevels.FOOT_SUPPORT_WEIGHT);
      spatialAccelerationCommand.set(rootBody, contactableFoot.getRigidBody());
      spatialAccelerationCommand.setPrimaryBase(pelvis);

      YoGraphicsListRegistry graphicsListRegistry = footControlHelper.getHighLevelHumanoidControllerToolbox().getYoGraphicsListRegistry();
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
      FrameVector3D fullyConstrainedNormalContactVector = footControlHelper.getFullyConstrainedNormalContactVector();
      controllerToolbox.setFootContactStateNormalContactVector(robotSide, fullyConstrainedNormalContactVector);

      computeFootPolygon();
   }

   @Override
   public void onExit()
   {
      if (frameViz != null)
         frameViz.hide();
   }

   @Override
   public void doAction(double timeInState)
   {
      computeFootPolygon();

      footSwitch.computeAndPackCoP(cop2d);
      if (cop2d.containsNaN())
         cop2d.setToZero(contactableFoot.getSoleFrame());

      YoPlaneContactState planeContactState = controllerToolbox.getFootContactState(robotSide);
      for (int i = 0; i < planeContactState.getTotalNumberOfContactPoints(); i++)
      {
         YoContactPoint contactPoint = planeContactState.getContactPoints().get(i);
         planeContactState.setMaxContactPointNormalForce(contactPoint, Double.POSITIVE_INFINITY);
      }

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

      // set selection matrices
      MovingReferenceFrame soleZUpFrame = controllerToolbox.getReferenceFrames().getSoleZUpFrame(robotSide);
      accelerationSelectionMatrix.resetSelection();
      accelerationSelectionMatrix.setSelectionFrame(soleZUpFrame);

      spatialAccelerationCommand.setSelectionMatrix(accelerationSelectionMatrix);

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

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return spatialAccelerationCommand;
   }

   public SpatialFeedbackControlCommand getFeedbackControlCommand()
   {
      return null;
   }

   public void setWeights(Vector3DReadOnly angularWeight, Vector3DReadOnly linearWeight)
   {
      this.angularWeight = angularWeight;
      this.linearWeight = linearWeight;
   }
}
