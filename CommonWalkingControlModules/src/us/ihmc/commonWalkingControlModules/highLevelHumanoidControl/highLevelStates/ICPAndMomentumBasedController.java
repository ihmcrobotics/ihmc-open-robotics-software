package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.FootPolygonVisualizer;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.calculators.ConstantOmega0Calculator;
import us.ihmc.commonWalkingControlModules.calculators.Omega0Calculator;
import us.ihmc.commonWalkingControlModules.calculators.Omega0CalculatorInterface;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPBasedLinearMomentumRateOfChangeControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.packetProducers.CapturabilityBasedStatusProducer;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.humanoidRobot.model.FullRobotModel;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;
import us.ihmc.yoUtilities.math.frames.YoFrameVector2d;

public class ICPAndMomentumBasedController
{
   private static final boolean USE_CONSTANT_OMEGA0 = true;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame centerOfMassFrame;
   private final MomentumBasedController momentumBasedController;

   private final SideDependentList<? extends ContactablePlaneBody> contactableFeet;
   private final BipedSupportPolygons bipedSupportPolygons;
   private final YoFramePoint2d yoDesiredCapturePoint = new YoFramePoint2d("desiredICP", "", worldFrame, registry);
   private final YoFrameVector2d yoDesiredICPVelocity = new YoFrameVector2d("desiredICPVelocity", "", worldFrame, registry);
   private final EnumYoVariable<RobotSide> supportLeg = new EnumYoVariable<>("supportLeg", registry, RobotSide.class, true);
   private final DoubleYoVariable controlledCoMHeightAcceleration = new DoubleYoVariable("controlledCoMHeightAcceleration", registry);
   private final YoFramePoint yoCapturePoint = new YoFramePoint("capturePoint", worldFrame, registry);
   private final DoubleYoVariable omega0 = new DoubleYoVariable("omega0", registry);
   private final Omega0CalculatorInterface omega0Calculator;


   private final FramePoint centerOfMassPosition = new FramePoint(worldFrame);
   private final FrameVector centerOfMassVelocity = new FrameVector(worldFrame);
   private final FramePoint2d centerOfMassPosition2d = new FramePoint2d(worldFrame);
   private final FrameVector2d centerOfMassVelocity2d = new FrameVector2d(worldFrame);

   private final FramePoint2d capturePoint2d = new FramePoint2d(worldFrame);
   private final FramePoint2d desiredCapturePoint2d = new FramePoint2d();
   private final FrameVector2d desiredCapturePointVelocity2d = new FrameVector2d();

   private final SpatialForceVector admissibleGroundReactionWrench = new SpatialForceVector();

   private final SideDependentList<YoPlaneContactState> footContactStates = new SideDependentList<YoPlaneContactState>();

   private final CapturabilityBasedStatusProducer capturabilityBasedStatusProducer;

   private final SpatialForceVector gravitationalWrench;

   private final SideDependentList<FramePoint2d> cops = new SideDependentList<>();

   private final ICPBasedLinearMomentumRateOfChangeControlModule icpBasedLinearMomentumRateOfChangeControlModule;

   private final MomentumRateOfChangeData momentumRateOfChangeData;

   public ICPAndMomentumBasedController(MomentumBasedController momentumBasedController, double omega0,
         ICPBasedLinearMomentumRateOfChangeControlModule icpBasedLinearMomentumRateOfChangeControlModule, BipedSupportPolygons bipedSupportPolygons,
         CapturabilityBasedStatusProducer capturabilityBasedStatusProducer, YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);

      this.momentumBasedController = momentumBasedController;
      this.capturabilityBasedStatusProducer = capturabilityBasedStatusProducer;
      this.icpBasedLinearMomentumRateOfChangeControlModule = icpBasedLinearMomentumRateOfChangeControlModule;

      FullRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();
      double totalMass = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator());

      double gravityZ = momentumBasedController.getGravityZ();
      centerOfMassFrame = momentumBasedController.getCenterOfMassFrame();
      gravitationalWrench = new SpatialForceVector(centerOfMassFrame, new Vector3d(0.0, 0.0, totalMass * gravityZ), new Vector3d());

      momentumRateOfChangeData = new MomentumRateOfChangeData(centerOfMassFrame);
      
      if (USE_CONSTANT_OMEGA0)
      {
         this.omega0Calculator = new ConstantOmega0Calculator(omega0, registry);
      }
      else
      {
         this.omega0Calculator = new Omega0Calculator(centerOfMassFrame, totalMass, omega0);
      }

      this.contactableFeet = momentumBasedController.getContactableFeet();
      this.bipedSupportPolygons = bipedSupportPolygons;

      for (RobotSide robotSide : RobotSide.values)
      {
         YoPlaneContactState footContactState = momentumBasedController.getContactState(contactableFeet.get(robotSide));
         footContactStates.put(robotSide, footContactState);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePoint2d cop = new FramePoint2d();
         cop.setToZero(contactableFeet.get(robotSide).getSoleFrame());
         cops.put(robotSide, cop);
      }

      // TODO: Have local updatables, instead of adding them to the momentum based controller.

      YoGraphicsListRegistry yoGraphicsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();

      if (yoGraphicsListRegistry != null)
      {
         ArrayList<PlaneContactState> feetContactStates = new ArrayList<PlaneContactState>();
         momentumBasedController.getFeetContactStates(feetContactStates);
         //         Collection<PlaneContactState> planeContactStates = momentumBasedController.getPlaneContactStates();
         FootPolygonVisualizer footPolygonVisualizer = new FootPolygonVisualizer(feetContactStates, yoGraphicsListRegistry, registry);
         momentumBasedController.addUpdatable(footPolygonVisualizer);

         YoGraphicPosition capturePointViz = new YoGraphicPosition("Capture Point", yoCapturePoint, 0.01, YoAppearance.Blue(), GraphicType.ROTATED_CROSS);
         yoGraphicsListRegistry.registerYoGraphic("Capture Point", capturePointViz);
         yoGraphicsListRegistry.registerArtifact("Capture Point", capturePointViz.createArtifact());
      }
   }

   public void initialize()
   {
      update();
   }

   public double getOmega0()
   {
      return omega0.getDoubleValue();
   }

   /**
    * Update the basics: capture point, omega0, and the support polygons.
    */
   public void update()
   {
      computeOmega0();
      computeCapturePoint();
      updateBipedSupportPolygons();
      icpBasedLinearMomentumRateOfChangeControlModule.updateCenterOfMassViz();
   }

   private void computeCapturePoint()
   {
      centerOfMassPosition.setToZero(centerOfMassFrame);
      momentumBasedController.getCenterOfMassVelocity(centerOfMassVelocity);

      centerOfMassPosition.changeFrame(worldFrame);
      centerOfMassVelocity.changeFrame(worldFrame);

      centerOfMassPosition2d.setByProjectionOntoXYPlane(centerOfMassPosition);
      centerOfMassVelocity2d.setByProjectionOntoXYPlane(centerOfMassVelocity);

      CapturePointCalculator.computeCapturePoint(capturePoint2d, centerOfMassPosition2d, centerOfMassVelocity2d, getOmega0());

      capturePoint2d.changeFrame(yoCapturePoint.getReferenceFrame());
      yoCapturePoint.setXY(capturePoint2d);
   }

   protected void updateBipedSupportPolygons()
   {
      bipedSupportPolygons.updateUsingContactStates(footContactStates);

      if (capturabilityBasedStatusProducer != null)
      {
         yoDesiredCapturePoint.getFrameTuple2dIncludingFrame(desiredCapturePoint2d);
         centerOfMassPosition.setToZero(centerOfMassFrame);
         centerOfMassPosition.changeFrame(worldFrame);
         capturabilityBasedStatusProducer.sendStatus(capturePoint2d, desiredCapturePoint2d, centerOfMassPosition, bipedSupportPolygons.getFootPolygonsInWorldFrame());
      }
   }

   private void computeOmega0()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         FramePoint2d desiredCoP = momentumBasedController.getDesiredCoP(contactableFeet.get(robotSide));
         if (desiredCoP != null)
            cops.get(robotSide).setIncludingFrame(desiredCoP);
      }

      momentumBasedController.getAdmissibleDesiredGroundReactionWrench(admissibleGroundReactionWrench);

      if (admissibleGroundReactionWrench.getLinearPartZ() == 0.0)
         admissibleGroundReactionWrench.set(gravitationalWrench); // FIXME: hack to resolve circularity

      omega0.set(omega0Calculator.computeOmega0(cops, admissibleGroundReactionWrench));
   }

   public void computeAndSubmitDesiredRateOfChangeOfMomentum(FramePoint2d finalDesiredCapturePoint2d, boolean keepCMPInsideSupportPolygon)
   {
      yoCapturePoint.getFrameTuple2dIncludingFrame(capturePoint2d);
      yoDesiredCapturePoint.getFrameTuple2dIncludingFrame(desiredCapturePoint2d);
      yoDesiredICPVelocity.getFrameTuple2dIncludingFrame(desiredCapturePointVelocity2d);

      icpBasedLinearMomentumRateOfChangeControlModule.keepCMPInsideSupportPolygon(keepCMPInsideSupportPolygon);
      
      icpBasedLinearMomentumRateOfChangeControlModule.setCapturePoint(capturePoint2d);
      icpBasedLinearMomentumRateOfChangeControlModule.setOmega0(omega0.getDoubleValue());

      icpBasedLinearMomentumRateOfChangeControlModule.setDesiredCapturePoint(desiredCapturePoint2d);
      icpBasedLinearMomentumRateOfChangeControlModule.setFinalDesiredCapturePoint(finalDesiredCapturePoint2d);
      icpBasedLinearMomentumRateOfChangeControlModule.setDesiredCapturePointVelocity(desiredCapturePointVelocity2d);

      icpBasedLinearMomentumRateOfChangeControlModule.setSupportLeg(supportLeg.getEnumValue());

      icpBasedLinearMomentumRateOfChangeControlModule.setDesiredCenterOfMassHeightAcceleration(controlledCoMHeightAcceleration.getDoubleValue());

      icpBasedLinearMomentumRateOfChangeControlModule.compute();
      icpBasedLinearMomentumRateOfChangeControlModule.getMomentumRateOfChange(momentumRateOfChangeData);
      momentumBasedController.setDesiredRateOfChangeOfMomentum(momentumRateOfChangeData);
   }

   public EnumYoVariable<RobotSide> getYoSupportLeg()
   {
      return supportLeg;
   }

   public DoubleYoVariable getControlledCoMHeightAcceleration()
   {
      return controlledCoMHeightAcceleration;
   }

   public BipedSupportPolygons getBipedSupportPolygons()
   {
      return bipedSupportPolygons;
   }

   public YoFramePoint getCapturePoint()
   {
      return yoCapturePoint;
   }

   public YoFramePoint2d getDesiredICP()
   {
      return yoDesiredCapturePoint;
   }

   public void getDesiredICP(FramePoint2d pointToPack)
   {
      this.yoDesiredCapturePoint.getFrameTuple2d(pointToPack);
   }

   public YoFrameVector2d getDesiredICPVelocity()
   {
      return yoDesiredICPVelocity;
   }

   public void getDesiredCMP(FramePoint2d desiredCMPToPack)
   {
      icpBasedLinearMomentumRateOfChangeControlModule.getDesiredCMP(desiredCMPToPack);
   }

   public SideDependentList<? extends ContactablePlaneBody> getBipedFeet()
   {
      return contactableFeet;
   }
}
