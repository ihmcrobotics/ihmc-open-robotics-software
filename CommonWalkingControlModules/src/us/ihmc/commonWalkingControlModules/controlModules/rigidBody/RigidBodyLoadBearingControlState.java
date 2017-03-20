package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;

public class RigidBodyLoadBearingControlState extends RigidBodyControlState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final long NO_CONTACT_ID = 0L;
   private static final long IN_CONTACT_ID = 1L;

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
   private final ReferenceFrame contactFrame;
   private final RigidBody body;
   private final ContactablePlaneBody contactableBody;

   private final RigidBodyTransform bodyToJointTransform = new RigidBodyTransform();
   private final RigidBodyTransform contactToJointTransform = new RigidBodyTransform();

   private final ArrayList<YoGraphic> graphics = new ArrayList<>();

   public RigidBodyLoadBearingControlState(RigidBody bodyToControl, ContactablePlaneBody contactableBody, RigidBody elevator, DoubleYoVariable yoTime,
         YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      super(RigidBodyControlMode.LOADBEARING, bodyToControl.getName(), yoTime, parentRegistry);
      this.body = bodyToControl;
      this.bodyFrame = bodyToControl.getBodyFixedFrame();
      this.elevatorFrame = elevator.getBodyFixedFrame();
      this.contactFrame = contactableBody.getSoleFrame();
      this.contactableBody = contactableBody;

      body.getBodyFixedFrame().getTransformToDesiredFrame(bodyToJointTransform, body.getParentJoint().getFrameAfterJoint());

      bodyAcceleration = new SpatialAccelerationVector(bodyFrame, elevatorFrame, bodyFrame);
      spatialAccelerationCommand.set(elevator, bodyToControl);
      spatialAccelerationCommand.setSelectionMatrixToIdentity();

      String bodyName = bodyToControl.getName();
      coefficientOfFriction = new DoubleYoVariable(bodyName + "CoefficientOfFriction", registry);
      contactNormal = new YoFrameVector(bodyName + "ContactNormal", worldFrame, parentRegistry);
      contactPoint = new YoFramePoint(bodyName + "ContactPoint", contactFrame, parentRegistry);

      planeContactStateCommand.setContactingRigidBody(body);
      planeContactStateCommand.setId(NO_CONTACT_ID);

      setupViz(graphicsListRegistry);
   }

   private void setupViz(YoGraphicsListRegistry graphicsListRegistry)
   {
      if (graphicsListRegistry == null)
         return;

      String listName = getClass().getSimpleName();
      YoGraphicReferenceFrame contactFrameViz = new YoGraphicReferenceFrame(contactFrame, registry, 0.1);
      graphicsListRegistry.registerYoGraphic(listName, contactFrameViz);
      graphics.add(contactFrameViz);
   }

   public void setCoefficientOfFriction(double coefficientOfFriction)
   {
      this.coefficientOfFriction.set(coefficientOfFriction);
   }

   public void setContactNormalInWorldFrame(Vector3D contactNormalInWorldFrame)
   {
      contactNormal.set(contactNormalInWorldFrame);
   }

   public void setContactFrame(RigidBodyTransform bodyToContactFrame)
   {
      contactToJointTransform.set(bodyToJointTransform);
      contactToJointTransform.multiply(bodyToContactFrame);
      contactableBody.setSoleFrameTransformFromParentJoint(contactToJointTransform);
      contactPoint.setToZero();
   }

   @Override
   public void doAction()
   {
      planeContactStateCommand.clearContactPoints();
      planeContactStateCommand.setCoefficientOfFriction(coefficientOfFriction.getDoubleValue());
      planeContactStateCommand.setContactNormal(contactNormal.getFrameTuple());
      planeContactStateCommand.addPointInContact(contactPoint.getFrameTuple());
      planeContactStateCommand.setContactingRigidBody(body);
      planeContactStateCommand.setId(IN_CONTACT_ID);

      bodyAcceleration.setToZero(contactFrame, elevatorFrame, contactFrame);
      bodyAcceleration.changeBodyFrameNoRelativeAcceleration(bodyFrame);
      bodyAcceleration.changeFrameNoRelativeMotion(bodyFrame);
      spatialAccelerationCommand.setSpatialAcceleration(bodyAcceleration);

      updateGraphics();
   }

   @Override
   public void doTransitionIntoAction()
   {
   }

   @Override
   public void doTransitionOutOfAction()
   {
      hideGraphics();
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      inverseDynamicsCommandList.clear();
      inverseDynamicsCommandList.addCommand(spatialAccelerationCommand);
      inverseDynamicsCommandList.addCommand(planeContactStateCommand);
      return inverseDynamicsCommandList;
   }

   public InverseDynamicsCommand<?> getEmptyPlaneContactStateCommand()
   {
      planeContactStateCommand.clearContactPoints();
      planeContactStateCommand.setContactingRigidBody(body);
      planeContactStateCommand.setId(NO_CONTACT_ID);

      updateGraphics();
      return planeContactStateCommand;
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

   private void updateGraphics()
   {
      for (int graphicsIdx = 0; graphicsIdx < graphics.size(); graphicsIdx++)
      {
         graphics.get(graphicsIdx).showGraphicObject();
         graphics.get(graphicsIdx).update();
      }
   }

   private void hideGraphics()
   {
      for (int graphicsIdx = 0; graphicsIdx < graphics.size(); graphicsIdx++)
         graphics.get(graphicsIdx).hideGraphicObject();
   }

}
