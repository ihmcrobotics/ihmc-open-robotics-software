package us.ihmc.avatar.networkProcessor.walkingPreview;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.controllers.pidGains.GainCalculator;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

public class WalkingPreviewContactPointHolder
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final PlaneContactState contactStateToHold;
   private final MovingReferenceFrame currentPlaneFrame;
   private final ReferenceFrame desiredPlaneFrame;

   private final double kp = 100.0;
   private final double zeta = 1.0;
   private final double kd = GainCalculator.computeDerivativeGain(kp, zeta);
   private final double weight = 50.0;

   private final InverseDynamicsCommandList commandList = new InverseDynamicsCommandList();

   public static WalkingPreviewContactPointHolder holdAtCurrent(PlaneContactState contactStateToHold)
   {
      FramePose3D desiredPose = new FramePose3D(contactStateToHold.getPlaneFrame());
      desiredPose.changeFrame(worldFrame);
      return new WalkingPreviewContactPointHolder(contactStateToHold, desiredPose);
   }

   public WalkingPreviewContactPointHolder(PlaneContactState contactStateToHold, FramePose3DReadOnly desiredPlaneFramePose)
   {
      this.contactStateToHold = contactStateToHold;
      currentPlaneFrame = (MovingReferenceFrame) contactStateToHold.getPlaneFrame();

      desiredPlaneFramePose.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      desiredPlaneFrame = new PoseReferenceFrame("desiredPlaneFrame", desiredPlaneFramePose);
   }

   public void doControl()
   {
      commandList.clear();

      for (FramePoint3D currentContactPoint : contactStateToHold.getContactFramePointsInContactCopy())
      {
         currentContactPoint.changeFrame(currentPlaneFrame);
         FrameVector3D toContactPoint = new FrameVector3D(currentContactPoint);
         toContactPoint.changeFrame(desiredPlaneFrame);
         FramePoint3D desiredContactPoint = new FramePoint3D(toContactPoint);
         PoseReferenceFrame controlFrame = new PoseReferenceFrame("atContactPoint", currentPlaneFrame);
         controlFrame.setPositionAndUpdate(currentContactPoint);

         FrameVector3D currentLinearVelocity = new FrameVector3D();
         currentPlaneFrame.getTwistOfFrame().getLinearVelocityAt(currentContactPoint, currentLinearVelocity);

         currentContactPoint.changeFrame(worldFrame);
         desiredContactPoint.changeFrame(worldFrame);
         currentLinearVelocity.changeFrame(worldFrame);

         FrameVector3D desiredLinearAcceleration = new FrameVector3D();
         desiredLinearAcceleration.sub(desiredContactPoint, currentContactPoint);
         desiredLinearAcceleration.scale(kp);
         desiredLinearAcceleration.scaleAdd(kd, currentLinearVelocity, desiredLinearAcceleration);

         SpatialAccelerationCommand command = new SpatialAccelerationCommand();
         command.set(MultiBodySystemTools.getRootBody(contactStateToHold.getRigidBody()), contactStateToHold.getRigidBody());
         command.setLinearAcceleration(controlFrame, desiredLinearAcceleration);
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
