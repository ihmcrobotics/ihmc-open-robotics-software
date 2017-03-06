package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;

public class FootControlHelper
{
   private static final double EPSILON_POINT_ON_EDGE = 5e-3;
   private static final double EPSILON_POINT_ON_EDGE_WITH_HYSTERESIS = 8e-3;

   private final RobotSide robotSide;
   private final ContactableFoot contactableFoot;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WalkingControllerParameters walkingControllerParameters;
   private final PartialFootholdControlModule partialFootholdControlModule;

   private final FrameVector fullyConstrainedNormalContactVector;
   private final BooleanYoVariable isDesiredCoPOnEdge;

   private final BipedSupportPolygons bipedSupportPolygons;

   private final LegSingularityAndKneeCollapseAvoidanceControlModule legSingularityAndKneeCollapseAvoidanceControlModule;

   public FootControlHelper(RobotSide robotSide, WalkingControllerParameters walkingControllerParameters, HighLevelHumanoidControllerToolbox controllerToolbox,
         YoVariableRegistry registry)
   {
      this.robotSide = robotSide;
      this.controllerToolbox = controllerToolbox;
      this.walkingControllerParameters = walkingControllerParameters;

      contactableFoot = controllerToolbox.getContactableFeet().get(robotSide);
      RigidBody foot = contactableFoot.getRigidBody();
      String namePrefix = foot.getName();

      YoGraphicsListRegistry yoGraphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();
      if (walkingControllerParameters.getOrCreateExplorationParameters(registry) != null)
      {
         partialFootholdControlModule = new PartialFootholdControlModule(robotSide, controllerToolbox,
               walkingControllerParameters, registry, yoGraphicsListRegistry);
      }
      else
      {
         partialFootholdControlModule = null;
      }

      isDesiredCoPOnEdge = new BooleanYoVariable(namePrefix + "IsDesiredCoPOnEdge", registry);

      fullyConstrainedNormalContactVector = new FrameVector(contactableFoot.getSoleFrame(), 0.0, 0.0, 1.0);

      bipedSupportPolygons = controllerToolbox.getBipedSupportPolygons();

      legSingularityAndKneeCollapseAvoidanceControlModule = new LegSingularityAndKneeCollapseAvoidanceControlModule(namePrefix, contactableFoot, robotSide,
            walkingControllerParameters, controllerToolbox, registry);
   }

   private final FramePoint2d desiredCoP = new FramePoint2d();

   public void update()
   {
      controllerToolbox.getDesiredCenterOfPressure(contactableFoot, desiredCoP);

      if (desiredCoP.containsNaN())
         isDesiredCoPOnEdge.set(false);
      else
      {
         double epsilon = isDesiredCoPOnEdge.getBooleanValue() ? EPSILON_POINT_ON_EDGE_WITH_HYSTERESIS : EPSILON_POINT_ON_EDGE;
         FrameConvexPolygon2d footSupportPolygon = bipedSupportPolygons.getFootPolygonInSoleFrame(robotSide);
         isDesiredCoPOnEdge.set(!footSupportPolygon.isPointInside(desiredCoP, -epsilon)); // Minus means that the check is done with a smaller polygon
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

   public HighLevelHumanoidControllerToolbox getHighLevelHumanoidControllerToolbox()
   {
      return controllerToolbox;
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
