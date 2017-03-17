package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;

public class RigidBodyLoadBearingControlState extends RigidBodyControlState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();
   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();

   // TODO: allow multiple surface normals?
   private final PlaneContactStateCommand planeContactStateCommand = new PlaneContactStateCommand();

   // TODO: allow multiple contact points
   private final YoFramePoint contactPoint;

   private final SpatialAccelerationVector bodyAcceleration;
   private final DoubleYoVariable coefficientOfFriction;
   private final YoFrameVector contactNormal;
   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame elevatorFrame;

   public RigidBodyLoadBearingControlState(RigidBody bodyToControl, RigidBody elevator, DoubleYoVariable yoTime, YoVariableRegistry parentRegistry)
   {
      super(RigidBodyControlMode.LOADBEARING, bodyToControl.getName(), yoTime, parentRegistry);
      this.bodyFrame = bodyToControl.getBodyFixedFrame();
      this.elevatorFrame = elevator.getBodyFixedFrame();

      bodyAcceleration = new SpatialAccelerationVector(bodyFrame, elevatorFrame, bodyFrame);
      spatialAccelerationCommand.set(elevator, bodyToControl);
      spatialAccelerationCommand.setSelectionMatrixToIdentity();

      String bodyName = bodyToControl.getName();
      coefficientOfFriction = new DoubleYoVariable(bodyName + "CoefficientOfFriction", registry);
      contactNormal = new YoFrameVector(bodyName + "ContactNormal", worldFrame, parentRegistry);
      contactPoint = new YoFramePoint(bodyName + "ContactPoint", bodyFrame, parentRegistry);

      planeContactStateCommand.setContactingRigidBody(bodyToControl);
   }

   public void setCoefficientOfFriction(double coefficientOfFriction)
   {
      this.coefficientOfFriction.set(coefficientOfFriction);
   }

   public void setContactNormal(FrameVector contactNormal)
   {
      contactNormal.checkReferenceFrameMatch(worldFrame);
      this.contactNormal.set(contactNormal);
   }

   public void setContactPoint(FramePoint contactPoint)
   {
      contactPoint.checkReferenceFrameMatch(bodyFrame);
      this.contactPoint.set(contactPoint);
   }

   @Override
   public void doAction()
   {
      planeContactStateCommand.clearContactPoints();
      planeContactStateCommand.setCoefficientOfFriction(coefficientOfFriction.getDoubleValue());
      planeContactStateCommand.setContactNormal(contactNormal.getFrameTuple());
      planeContactStateCommand.addPointInContact(contactPoint.getFrameTuple());

      bodyAcceleration.setToZero(bodyFrame, elevatorFrame, bodyFrame);
      spatialAccelerationCommand.setSpatialAcceleration(bodyAcceleration);
   }

   @Override
   public void doTransitionIntoAction()
   {
   }

   @Override
   public void doTransitionOutOfAction()
   {
      contactNormal.setToNaN();
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      inverseDynamicsCommandList.clear();
      inverseDynamicsCommandList.addCommand(spatialAccelerationCommand);
      inverseDynamicsCommandList.addCommand(planeContactStateCommand);
      return inverseDynamicsCommandList;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return null;
   }

   @Override
   public boolean isEmpty()
   {
      // this control mode does not support command queuing
      return false;
   }

   @Override
   public double getLastTrajectoryPointTime()
   {
      // this control mode does not support command queuing
      return 0.0;
   }

}
