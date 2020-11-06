package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ContactWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commons.InterpolationTools;
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
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import static us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType.GEQ_INEQUALITY;
import static us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType.LEQ_INEQUALITY;

public class JumpingSupportFootState implements JumpingFootControlState
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

   private final FootSwitchInterface footSwitch;

   private final PoseReferenceFrame controlFrame;
   private final YoGraphicReferenceFrame frameViz;

   private Vector3DReadOnly angularWeight;
   private Vector3DReadOnly linearWeight;

   private final DoubleProvider loadingDuration;
   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();
   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();

   private final YoDouble maxWeightFractionPerFoot;

   private final DoubleProvider rhoMin;
   private final double robotWeightFz;
   private final ContactWrenchCommand maxWrenchCommand;
   private final ContactWrenchCommand minWrenchCommand;
   private final YoDouble minZForce;
   private final YoDouble maxZForce;

   private final int numberOfBasisVectors;

   private final SelectionMatrix6D accelerationSelectionMatrix = new SelectionMatrix6D();

   private final FramePoint2D cop2d = new FramePoint2D();
   private final FramePoint3D framePosition = new FramePoint3D();
   private final FrameQuaternion frameOrientation = new FrameQuaternion();


   public JumpingSupportFootState(JumpingFootControlHelper footControlHelper, YoRegistry parentRegistry)
   {
      this.footControlHelper = footControlHelper;
      contactableFoot = footControlHelper.getContactableFoot();

      controllerToolbox = footControlHelper.getJumpingControllerToolbox();

      rhoMin = () -> footControlHelper.getWalkingControllerParameters().getMomentumOptimizationSettings().getRhoMin();


      robotSide = footControlHelper.getRobotSide();
      FullHumanoidRobotModel fullRobotModel = footControlHelper.getJumpingControllerToolbox().getFullRobotModel();
      pelvis = fullRobotModel.getPelvis();
      rootBody = fullRobotModel.getElevator();

      String prefix = footControlHelper.getRobotSide().getLowerCaseName() + "Foot";
      registry = new YoRegistry(prefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);

      footSwitch = footControlHelper.getJumpingControllerToolbox().getFootSwitches().get(robotSide);
      controlFrame = new PoseReferenceFrame(prefix + "HoldPositionFrame", contactableFoot.getSoleFrame());
      loadingDuration = new DoubleParameter(prefix + "LoadingDuration", registry, 0.05);

      spatialAccelerationCommand.setWeight(SolverWeightLevels.FOOT_SUPPORT_WEIGHT);
      spatialAccelerationCommand.set(rootBody, contactableFoot.getRigidBody());
      spatialAccelerationCommand.setPrimaryBase(pelvis);

      minWrenchCommand = new ContactWrenchCommand(GEQ_INEQUALITY);
      maxWrenchCommand = new ContactWrenchCommand(LEQ_INEQUALITY);

      robotWeightFz = controllerToolbox.getFullRobotModel().getTotalMass() * controllerToolbox.getGravityZ();
      numberOfBasisVectors = footControlHelper.getWalkingControllerParameters().getMomentumOptimizationSettings().getNumberOfBasisVectorsPerContactPoint();

      setupWrenchCommand(maxWrenchCommand);
      setupWrenchCommand(minWrenchCommand);
      minZForce = new YoDouble(robotSide.getLowerCaseName() + "MinZForce", registry);
      maxZForce = new YoDouble(robotSide.getLowerCaseName() + "MaxZForce", registry);
      maxWeightFractionPerFoot = new YoDouble(robotSide.getLowerCaseName() + "MaxWeightFractionPerFoot", registry);
      maxWeightFractionPerFoot.set(2.0);
      minWrenchCommand.getWrench().setLinearPartZ(minZForce.getValue());

      setLoadedPercent(1.0);

      YoGraphicsListRegistry graphicsListRegistry = footControlHelper.getJumpingControllerToolbox().getYoGraphicsListRegistry();
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

   private final FrameVector3D contactNormal = new FrameVector3D();

   @Override
   public void onEntry()
   {
      contactNormal.setIncludingFrame(contactableFoot.getSoleFrame(), 0.0, 0.0, 1.0);
      controllerToolbox.setFootContactStateNormalContactVector(robotSide, contactNormal);
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
      footSwitch.computeAndPackCoP(cop2d);
      if (cop2d.containsNaN())
         cop2d.setToZero(contactableFoot.getSoleFrame());

      setLoadedPercent(Math.min(timeInState / loadingDuration.getValue(), 1.0));

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

   private void setupWrenchCommand(ContactWrenchCommand command)
   {
      command.setRigidBody(contactableFoot.getRigidBody());
      command.getSelectionMatrix().clearSelection();
      command.getSelectionMatrix().setSelectionFrame(ReferenceFrame.getWorldFrame());
      command.getSelectionMatrix().selectLinearZ(true);
      command.getWrench().setToZero(contactableFoot.getRigidBody().getBodyFixedFrame(), ReferenceFrame.getWorldFrame());
   }


   public void setLoadedPercent(double percentLoaded)
   {
      if (minWrenchCommand == null)
         return;
      maxZForce.set(InterpolationTools.linearInterpolate(minZForce.getDoubleValue(), maxWeightFractionPerFoot.getValue() * robotWeightFz, percentLoaded));

      // Make sure the max force is always a little larger then the min force required by the rhoMin value. This is to avoid sending conflicting constraints.
      maxZForce.set(Math.max(maxZForce.getValue(), computeMinZForceBasedOnRhoMin(rhoMin.getValue()) + 1.0E-5));

      updateWrenchCommands();
   }

   private void updateWrenchCommands()
   {
      // Make sure the max force is always a little larger then the min force. This is to avoid sending conflicting constraints.
      maxZForce.set(Math.max(maxZForce.getValue(), minZForce.getValue() + 1.0E-5));

      minWrenchCommand.getWrench().setLinearPartZ(minZForce.getValue());
      maxWrenchCommand.getWrench().setLinearPartZ(maxZForce.getValue());
   }

   private final FrameVector3D normalVector = new FrameVector3D();

   private double computeMinZForceBasedOnRhoMin(double rhoMin)
   {
      YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);

      contactState.getContactNormalFrameVector(normalVector);
      normalVector.changeFrame(ReferenceFrame.getWorldFrame());
      normalVector.normalize();

      double friction = contactState.getCoefficientOfFriction();
      int points = contactState.getNumberOfContactPointsInContact();

      return normalVector.getZ() * rhoMin * numberOfBasisVectors * points / Math.sqrt(1.0 + friction * friction);
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      inverseDynamicsCommandList.clear();

      inverseDynamicsCommandList.addCommand(maxWrenchCommand);
      inverseDynamicsCommandList.addCommand(minWrenchCommand);
      inverseDynamicsCommandList.addCommand(spatialAccelerationCommand);
      return inverseDynamicsCommandList;
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
