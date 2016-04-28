package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.TwistCalculator;

public class FootControlHelper
{
   private static final double EPSILON_POINT_ON_EDGE = 5e-3;

   private final RobotSide robotSide;
   private final ContactableFoot contactableFoot;
   private final MomentumBasedController momentumBasedController;
   private final TwistCalculator twistCalculator;
   private final WalkingControllerParameters walkingControllerParameters;
   private final PartialFootholdControlModule partialFootholdControlModule;

   private final FrameVector fullyConstrainedNormalContactVector;
   private final BooleanYoVariable isDesiredCoPOnEdge;

   private final BipedSupportPolygons bipedSupportPolygons;

   private final LegSingularityAndKneeCollapseAvoidanceControlModule legSingularityAndKneeCollapseAvoidanceControlModule;

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
      double controlDT = momentumBasedController.getControlDT();

      partialFootholdControlModule = new PartialFootholdControlModule(namePrefix, controlDT, contactableFoot, twistCalculator, walkingControllerParameters,
            registry, momentumBasedController.getDynamicGraphicObjectsListRegistry());

      isDesiredCoPOnEdge = new BooleanYoVariable(namePrefix + "IsDesiredCoPOnEdge", registry);

      fullyConstrainedNormalContactVector = new FrameVector(contactableFoot.getSoleFrame(), 0.0, 0.0, 1.0);

      bipedSupportPolygons = momentumBasedController.getBipedSupportPolygons();

      legSingularityAndKneeCollapseAvoidanceControlModule = new LegSingularityAndKneeCollapseAvoidanceControlModule(namePrefix, contactableFoot, robotSide,
            walkingControllerParameters, momentumBasedController, registry);
   }

   private final FramePoint2d desiredCoP = new FramePoint2d();

   public void update()
   {
      momentumBasedController.getDesiredCenterOfPressure(contactableFoot, desiredCoP);

      if (desiredCoP.containsNaN())
         isDesiredCoPOnEdge.set(false);
      else
      {
         FrameConvexPolygon2d footSupportPolygon = bipedSupportPolygons.getFootPolygonInSoleFrame(robotSide);
         isDesiredCoPOnEdge.set(!footSupportPolygon.isPointInside(desiredCoP, -EPSILON_POINT_ON_EDGE)); // Minus means that the check is done with a smaller polygon
      }
   }

   public boolean isCoPOnEdge()
   {
      return isDesiredCoPOnEdge.getBooleanValue();
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public ContactableFoot getContactableFoot()
   {
      return contactableFoot;
   }

   public MomentumBasedController getMomentumBasedController()
   {
      return momentumBasedController;
   }

   public WalkingControllerParameters getWalkingControllerParameters()
   {
      return walkingControllerParameters;
   }

   public PartialFootholdControlModule getPartialFootholdControlModule()
   {
      return partialFootholdControlModule;
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

   public LegSingularityAndKneeCollapseAvoidanceControlModule getLegSingularityAndKneeCollapseAvoidanceControlModule()
   {
      return legSingularityAndKneeCollapseAvoidanceControlModule;
   }
}
