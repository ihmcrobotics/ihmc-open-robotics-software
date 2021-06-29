package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.YoSwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.controlModules.SwingTrajectoryCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold.FootholdRotationParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class FootControlHelper
{
   private final RobotSide robotSide;
   private final ContactableFoot contactableFoot;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WalkingControllerParameters walkingControllerParameters;
   private final PartialFootholdControlModule partialFootholdControlModule;

   private final FrameVector3D fullyConstrainedNormalContactVector;
   private final YoBoolean isDesiredCoPOnEdge;

   private final BipedSupportPolygons bipedSupportPolygons;

   private final WorkspaceLimiterControlModule workspaceLimiterControlModule;

   private final ToeSlippingDetector toeSlippingDetector;

   private final ExplorationParameters explorationParameters;
   private final FootholdRotationParameters footholdRotationParameters;
   private final SupportStateParameters supportStateParameters;

   private final SwingTrajectoryCalculator swingTrajectoryCalculator;
   private final YoSwingTrajectoryParameters swingTrajectoryParameters;

   public FootControlHelper(String prefix,
                            RobotSide robotSide,
                            WalkingControllerParameters walkingControllerParameters,
                            YoSwingTrajectoryParameters swingTrajectoryParameters,
                            WorkspaceLimiterParameters workspaceLimiterParameters,
                            HighLevelHumanoidControllerToolbox controllerToolbox,
                            ExplorationParameters explorationParameters,
                            FootholdRotationParameters footholdRotationParameters,
                            SupportStateParameters supportStateParameters,
                            YoRegistry registry)
   {
      this.robotSide = robotSide;
      this.controllerToolbox = controllerToolbox;
      this.walkingControllerParameters = walkingControllerParameters;
      this.explorationParameters = explorationParameters;
      this.footholdRotationParameters = footholdRotationParameters;
      this.supportStateParameters = supportStateParameters;

      this.swingTrajectoryParameters = swingTrajectoryParameters;
      this.swingTrajectoryCalculator = new SwingTrajectoryCalculator(robotSide.getCamelCaseNameForStartOfExpression() + prefix, robotSide, controllerToolbox,
                                                                     walkingControllerParameters,
                                                                     swingTrajectoryParameters,
                                                                     registry);

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
         workspaceLimiterControlModule = new WorkspaceLimiterControlModule(namePrefix + prefix,
                                                                           contactableFoot,
                                                                           robotSide,
                                                                           workspaceLimiterParameters,
                                                                           walkingControllerParameters,
                                                                           controllerToolbox,
                                                                           registry);
      }
      else
      {
         workspaceLimiterControlModule = null;
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
         double epsilon = isDesiredCoPOnEdge.getBooleanValue() ?
               supportStateParameters.getCopOnEdgeEpsilonWithHysteresis() :
               supportStateParameters.getCopOnEdgeEpsilon();
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

   public WorkspaceLimiterControlModule getWorkspaceLimiterControlModule()
   {
      return workspaceLimiterControlModule;
   }

   public ToeSlippingDetector getToeSlippingDetector()
   {
      return toeSlippingDetector;
   }

   public ExplorationParameters getExplorationParameters()
   {
      return explorationParameters;
   }

   public FootholdRotationParameters getFootholdRotationParameters()
   {
      return footholdRotationParameters;
   }

   public SupportStateParameters getSupportStateParameters()
   {
      return supportStateParameters;
   }

   public YoSwingTrajectoryParameters getSwingTrajectoryParameters()
   {
      return swingTrajectoryParameters;
   }

   public SwingTrajectoryCalculator getSwingTrajectoryCalculator()
   {
      return swingTrajectoryCalculator;
   }
}
