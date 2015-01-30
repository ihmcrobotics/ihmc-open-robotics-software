package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;

public class FootControlHelper
{
   private static final double EPSILON_POINT_ON_EDGE = 1e-2;

   private final RobotSide robotSide;
   private final ContactablePlaneBody contactableFoot;
   private final MomentumBasedController momentumBasedController;
   private final TwistCalculator twistCalculator;
   private final RigidBodySpatialAccelerationControlModule accelerationControlModule;
   private final WalkingControllerParameters walkingControllerParameters;
   private final PartialFootholdControlModule partialFootholdControlModule;

   private final int jacobianId;
   private final GeometricJacobian jacobian;
   private final EnumYoVariable<ConstraintType> requestedState;
   private final FrameVector fullyConstrainedNormalContactVector;

   private final FrameConvexPolygon2d contactPolygon = new FrameConvexPolygon2d();

   public FootControlHelper(RobotSide robotSide, WalkingControllerParameters walkingControllerParameters, MomentumBasedController momentumBasedController,
         YoVariableRegistry registry)
   {
      this.robotSide = robotSide;
      this.momentumBasedController = momentumBasedController;
      this.walkingControllerParameters = walkingControllerParameters;

      contactableFoot = momentumBasedController.getContactableFeet().get(robotSide);
      twistCalculator = momentumBasedController.getTwistCalculator();

      RigidBody foot = contactableFoot.getRigidBody();
      String namePrefix = foot.getName();
      ReferenceFrame frameAfterAnkle = contactableFoot.getFrameAfterParentJoint();
      double controlDT = momentumBasedController.getControlDT();

      accelerationControlModule = new RigidBodySpatialAccelerationControlModule(namePrefix, twistCalculator, foot, frameAfterAnkle, controlDT, registry);

      partialFootholdControlModule = new PartialFootholdControlModule(namePrefix, controlDT, contactableFoot, twistCalculator, walkingControllerParameters,
            registry, momentumBasedController.getDynamicGraphicObjectsListRegistry());


      FullRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();
      RigidBody pelvis = fullRobotModel.getPelvis();
      jacobianId = momentumBasedController.getOrCreateGeometricJacobian(pelvis, foot, foot.getBodyFixedFrame());
      jacobian = momentumBasedController.getJacobian(jacobianId);

      requestedState = EnumYoVariable.create(namePrefix + "RequestedState", "", ConstraintType.class, registry, true);

      fullyConstrainedNormalContactVector = new FrameVector(contactableFoot.getSoleFrame(), 0.0, 0.0, 1.0);

      contactPolygon.setIncludingFrameAndUpdate(contactableFoot.getContactPoints2d());
   }

   protected boolean isCoPOnEdge()
   {
      FramePoint2d cop = momentumBasedController.getDesiredCoP(contactableFoot);

      if (cop == null || cop.containsNaN())
         return false;
      else
         return !contactPolygon.isPointInside(cop, EPSILON_POINT_ON_EDGE);
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public ContactablePlaneBody getContactableFoot()
   {
      return contactableFoot;
   }

   public MomentumBasedController getMomentumBasedController()
   {
      return momentumBasedController;
   }

   public TwistCalculator getTwistCalculator()
   {
      return twistCalculator;
   }

   public RigidBodySpatialAccelerationControlModule getAccelerationControlModule()
   {
      return accelerationControlModule;
   }

   public WalkingControllerParameters getWalkingControllerParameters()
   {
      return walkingControllerParameters;
   }

   public PartialFootholdControlModule getPartialFootholdControlModule()
   {
      return partialFootholdControlModule;
   }

   public int getJacobianId()
   {
      return jacobianId;
   }

   public GeometricJacobian getJacobian()
   {
      return jacobian;
   }

   public void requestState(ConstraintType requestedState)
   {
      this.requestedState.set(requestedState);
   }

   public ConstraintType getRequestedState()
   {
      return requestedState.getEnumValue();
   }

   public void setRequestedStateAsProcessed()
   {
      requestedState.set(null);
   }

   public void setFullyConstrainedNormalContactVector(FrameVector normalContactVector)
   {
      if (normalContactVector != null)
         fullyConstrainedNormalContactVector.setIncludingFrame(normalContactVector);
      else
         fullyConstrainedNormalContactVector.setIncludingFrame(contactableFoot.getSoleFrame(), 0.0, 0.0, 1.0);
   }

   public FrameVector getFullyConstrainedNormalContactVector()
   {
      return fullyConstrainedNormalContactVector;
   }
}
