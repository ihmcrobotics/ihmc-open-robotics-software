package us.ihmc.avatar.kinematicsSimulation;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.controllers.pidGains.GainCalculator;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

public class HumanoidKinematicsSimulationContactStateHolder
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final PlaneContactState contactStateToHold;
   private final MovingReferenceFrame currentPlaneFrame;
   private final ReferenceFrame desiredPlaneFrame;

   private final double kp = 500.0;
   private final double zeta = 1.0;
   private final double kd = GainCalculator.computeDerivativeGain(kp, zeta);
   private final double weight = 50.0;
   private final MutableReferenceFrame controlFrame;
   private final RecyclingArrayList<SpatialAccelerationCommand> spatialAccelerationCommands
           = new RecyclingArrayList<>(SpatialAccelerationCommand::new);

   private final InverseDynamicsCommandList commandList = new InverseDynamicsCommandList();
   private final FramePoint3D desiredContactPoint;

   public static HumanoidKinematicsSimulationContactStateHolder holdAtCurrent(PlaneContactState contactStateToHold)
   {
      FramePose3D desiredPose = new FramePose3D(contactStateToHold.getPlaneFrame());
      desiredPose.changeFrame(worldFrame);
      return new HumanoidKinematicsSimulationContactStateHolder(contactStateToHold, desiredPose);
   }

   public HumanoidKinematicsSimulationContactStateHolder(PlaneContactState contactStateToHold, FramePose3DReadOnly desiredPlaneFramePose)
   {
      this.contactStateToHold = contactStateToHold;
      currentPlaneFrame = (MovingReferenceFrame) contactStateToHold.getPlaneFrame();
      controlFrame = new MutableReferenceFrame(currentPlaneFrame);

      desiredPlaneFramePose.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      desiredPlaneFrame = new PoseReferenceFrame("desiredPlaneFrame", desiredPlaneFramePose);
      desiredContactPoint = new FramePoint3D(desiredPlaneFrame);
   }

   public void doControl()
   {
      commandList.clear();
      spatialAccelerationCommands.clear();

      for (FramePoint3D currentContactPoint : contactStateToHold.getContactFramePointsInContactCopy())
      {
         currentContactPoint.changeFrame(currentPlaneFrame);
         desiredContactPoint.setIncludingFrame(desiredPlaneFrame, currentContactPoint);
         controlFrame.getTransformToParent().getTranslation().set(currentContactPoint);
         controlFrame.getReferenceFrame().update();

         FrameVector3D currentLinearVelocity = new FrameVector3D();
         currentPlaneFrame.getTwistOfFrame().getLinearVelocityAt(currentContactPoint, currentLinearVelocity);

         currentContactPoint.changeFrame(worldFrame);
         desiredContactPoint.changeFrame(worldFrame);
         currentLinearVelocity.changeFrame(worldFrame);

         FrameVector3D desiredLinearAcceleration = new FrameVector3D();
         desiredLinearAcceleration.sub(desiredContactPoint, currentContactPoint);
         desiredLinearAcceleration.scale(kp);
         desiredLinearAcceleration.scaleAdd(-kd, currentLinearVelocity, desiredLinearAcceleration);
         desiredLinearAcceleration.changeFrame(controlFrame.getReferenceFrame());

         SpatialAccelerationCommand command = spatialAccelerationCommands.add();
         command.getDesiredLinearAcceleration().setToZero();
         command.getDesiredAngularAcceleration().setToZero();
         command.getWeightMatrix().clear();
         command.getSelectionMatrix().clearSelection();

         RigidBodyBasics contactingBody = contactStateToHold.getRigidBody();
         command.set(MultiBodySystemTools.getRootBody(contactingBody), contactingBody);
         command.setLinearAcceleration(controlFrame.getReferenceFrame(), desiredLinearAcceleration);
         command.setWeight(0.0, weight);
         command.setSelectionMatrixForLinearControl();
         commandList.addCommand(command);
      }
   }

   public InverseDynamicsCommand<?> getOutput()
   {
      return commandList;
   }
}
