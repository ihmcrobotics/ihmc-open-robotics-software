package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class FootControlHelper
{
   private static final double EPSILON_POINT_ON_EDGE = 5e-3;
   private static final double EPSILON_POINT_ON_EDGE_WITH_HYSTERESIS = 8e-3;

   private final RobotSide robotSide;
   private final ContactableFoot contactableFoot;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WalkingControllerParameters walkingControllerParameters;
   private final PartialFootholdControlModule partialFootholdControlModule;

   private final FrameVector3D fullyConstrainedNormalContactVector;
   private final YoBoolean isDesiredCoPOnEdge;

   private final BipedSupportPolygons bipedSupportPolygons;

   private final LegSingularityAndKneeCollapseAvoidanceControlModule legSingularityAndKneeCollapseAvoidanceControlModule;

   private final ToeSlippingDetector toeSlippingDetector;

   private final ExplorationParameters explorationParameters;

   public FootControlHelper(RobotSide robotSide, WalkingControllerParameters walkingControllerParameters, HighLevelHumanoidControllerToolbox controllerToolbox,
                            ExplorationParameters explorationParameters, YoVariableRegistry registry)
   {
      this.robotSide = robotSide;
      this.controllerToolbox = controllerToolbox;
      this.walkingControllerParameters = walkingControllerParameters;
      this.explorationParameters = explorationParameters;

      contactableFoot = controllerToolbox.getContactableFeet().get(robotSide);
      RigidBodyBasics foot = contactableFoot.getRigidBody();
      String namePrefix = foot.getName();

      YoGraphicsListRegistry yoGraphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();
      if (walkingControllerParameters.createFootholdExplorationTools() && explorationParameters != null)
      {
         partialFootholdControlModule = new PartialFootholdControlModule(robotSide, controllerToolbox,
               walkingControllerParameters, explorationParameters, registry, yoGraphicsListRegistry);
      }
      else
      {
         partialFootholdControlModule = null;
      }

      isDesiredCoPOnEdge = new YoBoolean(namePrefix + "IsDesiredCoPOnEdge", registry);

      fullyConstrainedNormalContactVector = new FrameVector3D(contactableFoot.getSoleFrame(), 0.0, 0.0, 1.0);

      bipedSupportPolygons = controllerToolbox.getBipedSupportPolygons();

      if (walkingControllerParameters.enableLegSingularityAndKneeCollapseAvoidanceModule())
      {
         legSingularityAndKneeCollapseAvoidanceControlModule = new LegSingularityAndKneeCollapseAvoidanceControlModule(namePrefix, contactableFoot, robotSide,
                                                                                                                       walkingControllerParameters,
                                                                                                                       controllerToolbox, registry);
      }
      else
      {
         legSingularityAndKneeCollapseAvoidanceControlModule = null;
      }

      if (walkingControllerParameters.enableToeOffSlippingDetection())
      {
         double controlDT = controllerToolbox.getControlDT();
         FootSwitchInterface footSwitch = controllerToolbox.getFootSwitches().get(robotSide);
         toeSlippingDetector = new ToeSlippingDetector(namePrefix, controlDT, foot, footSwitch, registry);
         toeSlippingDetector.configure(walkingControllerParameters.getToeSlippingDetectorParameters());
      }
      else
      {
         toeSlippingDetector = null;
      }
   }

   private final FramePoint2D desiredCoP = new FramePoint2D();

   public void update()
   {
      controllerToolbox.getDesiredCenterOfPressure(contactableFoot, desiredCoP);

      if (desiredCoP.containsNaN())
         isDesiredCoPOnEdge.set(false);
      else
      {
         double epsilon = isDesiredCoPOnEdge.getBooleanValue() ? EPSILON_POINT_ON_EDGE_WITH_HYSTERESIS : EPSILON_POINT_ON_EDGE;
         FrameConvexPolygon2DReadOnly footSupportPolygon = bipedSupportPolygons.getFootPolygonInSoleFrame(robotSide);
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

   public ToeOffParameters getToeOffParameters()
   {
      return walkingControllerParameters.getToeOffParameters();
   }

   public SwingTrajectoryParameters getSwingTrajectoryParameters()
   {
      return walkingControllerParameters.getSwingTrajectoryParameters();
   }

   public PartialFootholdControlModule getPartialFootholdControlModule()
   {
      return partialFootholdControlModule;
   }

   public void setFullyConstrainedNormalContactVector(FrameVector3D normalContactVector)
   {
      if (normalContactVector != null)
         fullyConstrainedNormalContactVector.setIncludingFrame(normalContactVector);
      else
         fullyConstrainedNormalContactVector.setIncludingFrame(contactableFoot.getSoleFrame(), 0.0, 0.0, 1.0);
   }

   public FrameVector3D getFullyConstrainedNormalContactVector()
   {
      return fullyConstrainedNormalContactVector;
   }

   public LegSingularityAndKneeCollapseAvoidanceControlModule getLegSingularityAndKneeCollapseAvoidanceControlModule()
   {
      return legSingularityAndKneeCollapseAvoidanceControlModule;
   }

   public ToeSlippingDetector getToeSlippingDetector()
   {
      return toeSlippingDetector;
   }

   public ExplorationParameters getExplorationParameters()
   {
      return explorationParameters;
   }
}
