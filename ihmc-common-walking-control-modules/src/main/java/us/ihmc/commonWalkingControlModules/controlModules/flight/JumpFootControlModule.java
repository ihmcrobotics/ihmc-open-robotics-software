package us.ihmc.commonWalkingControlModules.controlModules.flight;

import java.util.Collection;
import java.util.Map;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.configurations.JumpControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyJointControlHelper;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyJointspaceControlState;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.weightMatrices.SolverWeightLevels;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class JumpFootControlModule
{
   private enum FootControlState
   {
      INVERSE_DYNAMICS, JOINTSPACE, TASKSPACE;
   }

   private final RobotSide robotSide;
   private final FootSwitchInterface footSwitch;
   private final ContactableFoot contactableFoot;

   private final PoseReferenceFrame controlFrame;
   private final FramePoint2D cop2d;
   private final FramePoint3D framePosition;
   private final FrameQuaternion frameOrientation;
   private final SpatialAccelerationCommand spatialAccelerationCommand;
   private final SpatialAccelerationVector footAcceleration;
   private final RigidBody rootBody;
   private final RigidBody pelvis;

   private final Wrench footWrench = new Wrench();

   private final RigidBodyTaskspaceControlState taskspaceControlState;
   private final RigidBodyJointspaceControlState jointspaceControlState;
   private final YoEnum<FootControlState> controlState;
   private PID3DGainsReadOnly inverseDynamicsOrientationGain;
   private PID3DGainsReadOnly inverseDynamicsPositionGain;
   private Vector3DReadOnly inverseDynamicsAngularWeight;
   private Vector3DReadOnly inverseDynamicsLinearWeight;

   public JumpFootControlModule(RobotSide robotSide, FootSwitchInterface footSwitch, ContactableFoot contactableFoot, RigidBody rootBody, RigidBody pelvis,
                                Collection<ReferenceFrame> trajectoryFrames, TObjectDoubleHashMap<String> homeConfiguration,
                                JumpControllerParameters jumpControlParameters, YoDouble yoTime, YoGraphicsListRegistry graphicsListRegistry,
                                YoVariableRegistry registry)
   {
      this.robotSide = robotSide;
      this.contactableFoot = contactableFoot;
      this.footSwitch = footSwitch;
      this.rootBody = rootBody;
      this.pelvis = pelvis;

      ReferenceFrame footSoleFrame = contactableFoot.getSoleFrame();
      this.cop2d = new FramePoint2D(footSoleFrame);
      this.framePosition = new FramePoint3D(footSoleFrame);
      this.frameOrientation = new FrameQuaternion(footSoleFrame);
      this.controlFrame = new PoseReferenceFrame(robotSide.getCamelCaseNameForStartOfExpression() + "ControlFrame", footSoleFrame);
      this.spatialAccelerationCommand = new SpatialAccelerationCommand();
      this.footAcceleration = new SpatialAccelerationVector();

      OneDoFJoint[] jointsToControl = ScrewTools.createOneDoFJointPath(pelvis, contactableFoot.getRigidBody());
      String rigidBodyName = contactableFoot.getName();
      RigidBodyJointControlHelper jointControlHelper = new RigidBodyJointControlHelper(rigidBodyName, jointsToControl, registry);
      taskspaceControlState = new RigidBodyTaskspaceControlState(rigidBodyName, contactableFoot.getRigidBody(), pelvis, rootBody, trajectoryFrames,
                                                                 footSoleFrame, pelvis.getBodyFixedFrame(), yoTime, jointControlHelper, graphicsListRegistry,
                                                                 registry);
      jointspaceControlState = new RigidBodyJointspaceControlState(rigidBodyName, jointsToControl, homeConfiguration, yoTime, jointControlHelper, registry);

      controlState = new YoEnum<>(contactableFoot.getName() + "ControlState", registry, FootControlState.class);
      controlState.set(FootControlState.INVERSE_DYNAMICS);
      setupSpatialAccelerationCommand();
   }

   public void setGains(Map<String, PIDGainsReadOnly> jointGainMap, PID3DGainsReadOnly taskspaceOrientationGains, PID3DGainsReadOnly taskspacePositionGains,
                        PID3DGainsReadOnly inverseDynamicsOrientationGain, PID3DGainsReadOnly inverseDynamicsPositionGain)
   {
      taskspaceControlState.setGains(taskspaceOrientationGains, taskspacePositionGains);
      jointspaceControlState.setGains(jointGainMap);
      this.inverseDynamicsPositionGain = inverseDynamicsPositionGain;
      this.inverseDynamicsOrientationGain = inverseDynamicsOrientationGain;
   }

   public void setWeights(Map<String, DoubleProvider> jointspaceWeights, Vector3DReadOnly taskspceAngularWeight, Vector3DReadOnly taskSpaceLinearWeight,
                          Vector3DReadOnly inverseDynamicsAngularWeight, Vector3DReadOnly inverseDynamicsLinearWeight)
   {
      taskspaceControlState.setWeights(taskspceAngularWeight, taskSpaceLinearWeight);
      jointspaceControlState.setDefaultWeights(jointspaceWeights);
      this.inverseDynamicsAngularWeight = inverseDynamicsAngularWeight; 
      this.inverseDynamicsLinearWeight = inverseDynamicsLinearWeight;
   }

   public double getGroundReactionForceZ()
   {
      footSwitch.computeAndPackFootWrench(footWrench);
      return footWrench.getLinearPartZ();
   }

   public void complyAndDamp()
   {
      controlState.set(FootControlState.INVERSE_DYNAMICS);
   }

   public void compute()
   {
      switch (controlState.getValue())
      {
      case INVERSE_DYNAMICS:
         computeForInverseDynamicsMode();
         break;
      case JOINTSPACE:
         jointspaceControlState.doAction();
         break;
      case TASKSPACE:
         taskspaceControlState.doAction();
         break;
      default:
         throw getExceptionForInvalidStateEnum();
      }
   }

   private void computeForInverseDynamicsMode()
   {
      footSwitch.computeAndPackCoP(cop2d);
      if (cop2d.containsNaN())
         cop2d.setToZero(contactableFoot.getSoleFrame());
      framePosition.setIncludingFrame(cop2d, 0.0);
      frameOrientation.setToZero(contactableFoot.getSoleFrame());
      controlFrame.setPoseAndUpdate(framePosition, frameOrientation);

      ReferenceFrame bodyFixedFrame = contactableFoot.getRigidBody().getBodyFixedFrame();
      footAcceleration.setToZero(bodyFixedFrame, rootBody.getBodyFixedFrame(), controlFrame);
      footAcceleration.changeBodyFrameNoRelativeAcceleration(bodyFixedFrame);
      spatialAccelerationCommand.setSpatialAcceleration(controlFrame, footAcceleration);
      if (inverseDynamicsLinearWeight != null && inverseDynamicsOrientationGain != null)
         spatialAccelerationCommand.setWeights(inverseDynamicsAngularWeight, inverseDynamicsLinearWeight);
   }

   public void holdInJointSpace()
   {
      controlState.set(FootControlState.JOINTSPACE);
      jointspaceControlState.holdCurrent();
   }

   public void holdInTaskspace()
   {
      controlState.set(FootControlState.TASKSPACE);
      taskspaceControlState.holdCurrent();
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      if (controlState.getValue() == FootControlState.INVERSE_DYNAMICS)
         return spatialAccelerationCommand;
      else
         return null;
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      switch (controlState.getValue())
      {
      case INVERSE_DYNAMICS:
         return null;
      case TASKSPACE:
         return taskspaceControlState.getFeedbackControlCommand();
      case JOINTSPACE:
         return jointspaceControlState.getFeedbackControlCommand();
      default:
         throw getExceptionForInvalidStateEnum();
      }
   }

   private RuntimeException getExceptionForInvalidStateEnum()
   {
      throw new RuntimeException("Unknown foot control state");
   }

   private void setupSpatialAccelerationCommand()
   {
      spatialAccelerationCommand.setWeight(SolverWeightLevels.FOOT_SUPPORT_WEIGHT);
      spatialAccelerationCommand.set(rootBody, contactableFoot.getRigidBody());
      spatialAccelerationCommand.setPrimaryBase(pelvis);
   }

   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      FeedbackControlCommandList templateFeedbackCommand = new FeedbackControlCommandList();
      templateFeedbackCommand.addCommand(taskspaceControlState.getFeedbackControlCommand());
      templateFeedbackCommand.addCommand(jointspaceControlState.getFeedbackControlCommand());
      return templateFeedbackCommand;
   }
   
   private final FramePoint3D tempPoint = new FramePoint3D();

   public double getFootPosition(Axis axis)
   {
      tempPoint.setToZero(contactableFoot.getSoleFrame());
      return tempPoint.getElement(axis.ordinal());
   }

}
