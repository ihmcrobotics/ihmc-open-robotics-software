package us.ihmc.commonWalkingControlModules.controlModules.dynamicLoadBearing;

import org.apache.commons.lang3.mutable.MutableBoolean;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.WholeBodyContactState;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;

public class DynamicLoadBearingPostContactState implements DynamicLoadBearingState
{
   private static final double COEFFICIENT_OF_FRICTION = 0.5;
   private static final double LOAD_TIME = 0.75;

   private final RigidBodyBasics bodyToControl;
   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame elevatorFrame;
   private final MutableBoolean hasContactChanged;

   /* Controller Commands */
   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();
   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();
   private final PlaneContactStateCommand planeContactStateCommand = new PlaneContactStateCommand();

   private final SelectionMatrix6D spatialAccelerationSelectionMatrix = new SelectionMatrix6D();
   private final SpatialAcceleration bodyAcceleration;
   private final PoseReferenceFrame desiredContactFrameFixedInWorld;

   private final FramePoint3D contactPointInBody = new FramePoint3D();
   private final FramePoint3D bracingContactPoint = new FramePoint3D();
   private final FrameVector3D bracingNormal = new FrameVector3D();

   public DynamicLoadBearingPostContactState(RigidBodyBasics bodyToControl,
                                             RigidBodyBasics baseBody,
                                             RigidBodyBasics elevator,
                                             ReferenceFrame controlFrame,
                                             MutableBoolean hasContactChanged,
                                             YoRegistry registry)
   {
      this.bodyToControl = bodyToControl;
      this.bodyFrame = bodyToControl.getBodyFixedFrame();
      this.elevatorFrame = elevator.getBodyFixedFrame();
      this.hasContactChanged = hasContactChanged;

      bracingContactPoint.setToZero(controlFrame);
      contactPointInBody.setToZero(controlFrame);
      contactPointInBody.changeFrame(bodyToControl.getBodyFixedFrame());

      String bodyName = bodyToControl.getName();
      planeContactStateCommand.setContactingRigidBody(bodyToControl);

      spatialAccelerationCommand.set(elevator, bodyToControl);
      spatialAccelerationCommand.setPrimaryBase(baseBody);

      desiredContactFrameFixedInWorld = new PoseReferenceFrame("desiredContactFrame" + bodyName, ReferenceFrame.getWorldFrame());
      bodyAcceleration = new SpatialAcceleration(desiredContactFrameFixedInWorld, elevatorFrame, desiredContactFrameFixedInWorld);
   }

   public void setBracingSurface(Vector3DReadOnly bracingNormal)
   {
      this.bracingNormal.set(bracingNormal);
   }

   /**
    * Assumes when this state is entered that the hand is in contact with the surface, to the best of the estimator's knowledge.
    * For simulation, this is just done with just distance to the planar region.
    * For real robot, maybe augment with some proprioceptive knowledge. Or maybe the map is good enough.
    *
    * Orientation is not controlled during pre-contact, but should be here (should it be?)
    */
   @Override
   public void onEntry()
   {
      hasContactChanged.setValue(true);
   }

   @Override
   public void doAction(double timeInState)
   {
      planeContactStateCommand.clearContactPoints();
      planeContactStateCommand.setCoefficientOfFriction(COEFFICIENT_OF_FRICTION);
      planeContactStateCommand.setContactNormal(bracingNormal);
      planeContactStateCommand.addPointInContact(bracingContactPoint);
      planeContactStateCommand.setHasContactStateChanged(false);

      bodyAcceleration.setToZero(desiredContactFrameFixedInWorld, elevatorFrame, desiredContactFrameFixedInWorld);
      bodyAcceleration.setBodyFrame(bodyFrame);
      spatialAccelerationCommand.setSpatialAcceleration(desiredContactFrameFixedInWorld, bodyAcceleration);
      spatialAccelerationSelectionMatrix.getAngularPart().clearSelection();
      spatialAccelerationSelectionMatrix.getLinearPart().selectAxis(Axis3D.X.ordinal(), true);
      spatialAccelerationSelectionMatrix.getLinearPart().selectAxis(Axis3D.Y.ordinal(), true);
      spatialAccelerationSelectionMatrix.getLinearPart().selectAxis(Axis3D.Z.ordinal(), true);
      spatialAccelerationCommand.setSelectionMatrix(spatialAccelerationSelectionMatrix);
      spatialAccelerationCommand.getWeightMatrix().getLinearPart().setWeights(50.0); // TODO parameterize
   }

   @Override
   public void onExit(double timeInState)
   {
      hasContactChanged.setValue(true);
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      inverseDynamicsCommandList.clear();
      inverseDynamicsCommandList.addCommand(planeContactStateCommand);
      inverseDynamicsCommandList.addCommand(spatialAccelerationCommand);
      return inverseDynamicsCommandList;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      // Maybe at point feedback later, but for now just use spatial acceleration command

      return null;
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      return null;
   }

   public InverseDynamicsCommand<?> getTransitionOutOfStateCommand()
   {
      planeContactStateCommand.clearContactPoints();
      planeContactStateCommand.setHasContactStateChanged(true);
      return planeContactStateCommand;
   }

   public void updateWholeBodyContactState(WholeBodyContactState wholeBodyContactStateToUpdate)
   {
      wholeBodyContactStateToUpdate.addContactPoints(planeContactStateCommand);
   }

   public void packContactData(RecyclingArrayList<Point3D> contactPointList, Vector3DBasics contactNormalToPack)
   {
      contactPointList.add().set(contactPointInBody);
      contactNormalToPack.set(bracingNormal);
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      return null;
   }
}
