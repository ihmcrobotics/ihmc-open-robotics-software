package us.ihmc.commonWalkingControlModules.controlModules.flight;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.JumpControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.weightMatrices.SolverWeightLevels;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class JumpFootControlModule
{
   private final RobotSide robotSide;
   private final YoPlaneContactState contactState;
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
   private final Vector3D angularWeight = new Vector3D();
   private final Vector3D linearWeight = new Vector3D();
   
   private final Wrench footWrench = new Wrench();
   
   public JumpFootControlModule(RobotSide robotSide, YoPlaneContactState footContactState, FootSwitchInterface footSwitch, ContactableFoot contactableFoot,
                                RigidBody rootBody, RigidBody pelvis, JumpControllerParameters jumpControlParameters, YoVariableRegistry registry)
   {
      this.robotSide = robotSide;
      this.contactState = footContactState;
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
      setupSpatialAccelerationCommand();
   }

   private void setupSpatialAccelerationCommand()
   {
      spatialAccelerationCommand.setWeight(SolverWeightLevels.FOOT_SUPPORT_WEIGHT);
      spatialAccelerationCommand.set(rootBody, contactableFoot.getRigidBody());
      spatialAccelerationCommand.setPrimaryBase(pelvis);
   }

   public void complyAndDamp()
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
      //spatialAccelerationCommand.setWeights(angularWeight, linearWeight);
   }

   public void holdPositionInJointSpace()
   {

   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return spatialAccelerationCommand;
   }

   public double getGroundReactionForceZ()
   {
      footSwitch.computeAndPackFootWrench(footWrench);
      return footWrench.getLinearPartZ();
   }

}
