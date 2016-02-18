package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlMode;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.TaskspaceToJointspaceCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.SpatialAccelerationCommand;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.PoseTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicReferenceFrame;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.tools.FormattingTools;

/**
 * @author twan
 *         Date: 5/9/13
 */
public class TaskspaceHandPositionControlState extends TrajectoryBasedTaskspaceHandControlState
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final SpatialAccelerationVector handAcceleration = new SpatialAccelerationVector();

   private final String name;
   private final YoVariableRegistry registry;

   private final RigidBody base;

   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();
   private final DenseMatrix64F selectionMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);

   // viz stuff:
   private final YoGraphicReferenceFrame dynamicGraphicReferenceFrame;
   private final PoseReferenceFrame desiredPositionFrame;

   // temp stuff:
   private final FramePose desiredPose = new FramePose();
   private final FramePoint desiredPosition = new FramePoint(worldFrame);
   private final FrameVector desiredVelocity = new FrameVector(worldFrame);
   private final FrameVector desiredAcceleration = new FrameVector(worldFrame);

   private final FrameOrientation desiredOrientation = new FrameOrientation(worldFrame);
   private final FrameVector desiredAngularVelocity = new FrameVector(worldFrame);
   private final FrameVector desiredAngularAcceleration = new FrameVector(worldFrame);

   private PoseTrajectoryGenerator poseTrajectoryGenerator;
   private RigidBodySpatialAccelerationControlModule handSpatialAccelerationControlModule;

   private final DoubleYoVariable doneTrajectoryTime;
   private final DoubleYoVariable holdPositionDuration;

   public TaskspaceHandPositionControlState(String namePrefix, HandControlMode stateEnum, MomentumBasedController momentumBasedController, int jacobianId,
         RigidBody base, RigidBody endEffector, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      super(stateEnum);

      name = namePrefix + FormattingTools.underscoredToCamelCase(this.getStateEnum().toString(), true) + "State";
      registry = new YoVariableRegistry(name);

      spatialAccelerationCommand.set(base, endEffector);
      GeometricJacobian jacobian = momentumBasedController.getJacobian(jacobianId);
      spatialAccelerationCommand.setJacobian(jacobian);

      this.base = base;

      parentRegistry.addChild(registry);

      desiredPositionFrame = new PoseReferenceFrame(name + "DesiredFrame", worldFrame);

      if (yoGraphicsListRegistry != null)
      {
         YoGraphicsList list = new YoGraphicsList(name);

         dynamicGraphicReferenceFrame = new YoGraphicReferenceFrame(desiredPositionFrame, registry, 0.3);
         list.add(dynamicGraphicReferenceFrame);

         yoGraphicsListRegistry.registerYoGraphicsList(list);
         list.hideYoGraphics();
      }
      else
      {
         dynamicGraphicReferenceFrame = null;
      }

      doneTrajectoryTime = new DoubleYoVariable(namePrefix + "DoneTrajectoryTime", registry);
      holdPositionDuration = new DoubleYoVariable(namePrefix + "HoldPositionDuration", registry);
   }

   @Override
   public void doAction()
   {
      if (poseTrajectoryGenerator.isDone())
         recordDoneTrajectoryTime();

      poseTrajectoryGenerator.compute(getTimeInCurrentState());

      poseTrajectoryGenerator.getPose(desiredPose);
      poseTrajectoryGenerator.getLinearData(desiredPosition, desiredVelocity, desiredAcceleration);
      poseTrajectoryGenerator.getAngularData(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);

      handSpatialAccelerationControlModule.doPositionControl(desiredPosition, desiredOrientation, desiredVelocity, desiredAngularVelocity, desiredAcceleration, desiredAngularAcceleration, base);

      handSpatialAccelerationControlModule.getAcceleration(handAcceleration);

      ReferenceFrame handFrame = handSpatialAccelerationControlModule.getEndEffector().getBodyFixedFrame();
      handAcceleration.changeBodyFrameNoRelativeAcceleration(handFrame);
      handAcceleration.changeFrameNoRelativeMotion(handFrame);

      updateVisualizers();
      spatialAccelerationCommand.set(handAcceleration);
   
   }

   @Override
   public void doTransitionIntoAction()
   {
      poseTrajectoryGenerator.showVisualization();
      poseTrajectoryGenerator.initialize();
      doneTrajectoryTime.set(Double.NaN);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      holdPositionDuration.set(0.0);
   }

   @Override
   public boolean isDone()
   {
      if (Double.isNaN(doneTrajectoryTime.getDoubleValue()))
         return false;

      return getTimeInCurrentState() > doneTrajectoryTime.getDoubleValue() + holdPositionDuration.getDoubleValue();
   }

   private void recordDoneTrajectoryTime()
   {
      if (Double.isNaN(doneTrajectoryTime.getDoubleValue()))
      {
         doneTrajectoryTime.set(getTimeInCurrentState());
      }
   }

   private void updateVisualizers()
   {
      if (dynamicGraphicReferenceFrame != null)
      {
         desiredPosition.changeFrame(worldFrame);
         desiredOrientation.changeFrame(worldFrame);
         desiredPositionFrame.setPoseAndUpdate(desiredPosition, desiredOrientation);
         desiredPositionFrame.update();

         dynamicGraphicReferenceFrame.update();
      }

      if (poseTrajectoryGenerator.isDone())
         poseTrajectoryGenerator.hideVisualization();
   }

   @Override
   public void setHoldPositionDuration(double time)
   {
      holdPositionDuration.set(time);
   }

   @Override
   public void setTrajectory(PoseTrajectoryGenerator poseTrajectoryGenerator)
   {
      this.poseTrajectoryGenerator = poseTrajectoryGenerator;
   }

   @Override
   public void setControlModuleForForceControl(RigidBodySpatialAccelerationControlModule handRigidBodySpatialAccelerationControlModule)
   {
      handSpatialAccelerationControlModule = handRigidBodySpatialAccelerationControlModule;
   }

   @Override
   public void setControlModuleForPositionControl(TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator)
   {
   }

   @Override
   public void setSelectionMatrix(DenseMatrix64F selectionMatrix)
   {
      this.selectionMatrix.reshape(selectionMatrix.getNumRows(), selectionMatrix.getNumCols());
      this.selectionMatrix.set(selectionMatrix);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return desiredPose.getReferenceFrame();
   }

   @Override
   public FramePose getDesiredPose()
   {
      return desiredPose;
   }

   @Override
   public SpatialAccelerationCommand getInverseDynamicsCommand()
   {
      return spatialAccelerationCommand;
   }
}
