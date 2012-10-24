package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.EnumMap;
import java.util.HashMap;

import javax.vecmath.Matrix3d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedFootInterface;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.FootPolygonVisualizer;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DesiredCoPAndCMPControlModule;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.LegStrengthCalculator;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.VirtualToePointCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.CenterOfMassHeightControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.NewGeometricVirtualToePointCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.PartialCenterOfMassHeightControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.SacrificeDeltaCMPDesiredCoPAndCMPControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.TeeterTotterLegStrengthCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP.CapturabilityBasedDesiredCoPVisualizer;
import us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP.SimpleDesiredCenterOfPressureFilter;
import us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP.SpeedControllingDesiredCoPCalculator;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.kinematics.SpatialAccelerationProjector;
import us.ihmc.commonWalkingControlModules.outputs.ProcessedOutputsInterface;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLine2d;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.GeometryTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.screwTheory.InverseDynamicsCalculator;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.TotalMassCalculator;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoAppearance;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition.GraphicType;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class MomentumBasedController implements RobotController
{
   private static final long serialVersionUID = -744968203951905486L;
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final ProcessedSensorsInterface processedSensors;

   private final ProcessedOutputsInterface processedOutputs;
   private final InverseDynamicsCalculator inverseDynamicsCalculator;

   private final BipedSupportPolygons bipedSupportPolygons;
   private final VirtualToePointCalculator virtualToePointCalculator;
   private final CenterOfMassHeightControlModule centerOfMassHeightControlModule;
   private final LegStrengthCalculator legStrengthCalculator;

   private final SideDependentList<BipedFootInterface> bipedFeet;
   private final ReferenceFrame midFeetZUp;
   private final double totalMass;

   private final HighLevelHumanoidController highLevelHumanoidController;
   private final BipedMomentumOptimizer optimizer;

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final FullRobotModel fullRobotModel;
   private final CommonWalkingReferenceFrames referenceFrames;

   private final SideDependentList<EnumMap<LimbName, SpatialAccelerationVector>> desiredEndEffectorAccelerationsInWorld =
      SideDependentList.createListOfEnumMaps(LimbName.class);
   private final SideDependentList<EndEffectorPoseTwistAndSpatialAccelerationCalculator> footPoseTwistAndSpatialAccelerationCalculators =
      new SideDependentList<EndEffectorPoseTwistAndSpatialAccelerationCalculator>();
   private final SideDependentList<SpatialAccelerationProjector> spatialAccelerationProjectors = new SideDependentList<SpatialAccelerationProjector>();
   private final SideDependentList<BooleanYoVariable> isCoPOnEdge = new SideDependentList<BooleanYoVariable>();
   private final SideDependentList<YoFramePoint> desiredFootPositionsInWorld = new SideDependentList<YoFramePoint>();

   private final DesiredHeadingControlModule desiredHeadingControlModule;
   private final DesiredCoPAndCMPControlModule desiredCoPAndCMPControlModule;

   private final ThreeDoFAngularAccelerationControlModule chestAngularAccelerationcalculator;

   private final YoFrameVector desiredPelvisLinearAcceleration;
   private final YoFrameVector desiredPelvisAngularAcceleration;
   private final YoFrameVector desiredPelvisForce;
   private final YoFrameVector desiredPelvisTorque;
   private final ReferenceFrame centerOfMassFrame;

   private final DoubleYoVariable kUpperBody = new DoubleYoVariable("kUpperBody", registry);
   private final DoubleYoVariable zetaUpperBody = new DoubleYoVariable("zetaUpperBody", registry);
   private final DoubleYoVariable kAngularMomentumZ = new DoubleYoVariable("kAngularMomentumZ", registry);
   private final DoubleYoVariable kPelvisYaw = new DoubleYoVariable("kPelvisYaw", registry);
   private final HashMap<RevoluteJoint, DoubleYoVariable> desiredAccelerationYoVariables = new HashMap<RevoluteJoint, DoubleYoVariable>();

   private final SideDependentList<Double> lambdas = new SideDependentList<Double>();

   private final FootPolygonVisualizer footPolygonVisualizer;
   private final DoubleYoVariable omega0 = new DoubleYoVariable("omega0", registry);
   private final YoFramePoint capturePoint = new YoFramePoint("capturePoint", worldFrame, registry);

   private final BooleanYoVariable leftInSingularRegion = new BooleanYoVariable("leftInSingularRegion", registry);
   private final BooleanYoVariable rightInSingularRegion = new BooleanYoVariable("rightInSingularRegion", registry);
   private final SideDependentList<BooleanYoVariable> inSingularRegions = new SideDependentList<BooleanYoVariable>(leftInSingularRegion, rightInSingularRegion);
   private boolean doChestOrientationControl;

   public MomentumBasedController(ProcessedSensorsInterface processedSensors, ProcessedOutputsInterface processedOutputs,
                                  CommonWalkingReferenceFrames referenceFrames, TwistCalculator twistCalculator, double controlDT,
                                  DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, SideDependentList<BipedFootInterface> bipedFeet,
                                  BipedSupportPolygons bipedSupportPolygons, DesiredHeadingControlModule desiredHeadingControlModule,
                                  HighLevelHumanoidController highLevelHumanoidController, boolean doChestOrientationControl)
   {
      this.processedSensors = processedSensors;
      this.fullRobotModel = processedSensors.getFullRobotModel();
      this.referenceFrames = referenceFrames;
      this.processedOutputs = processedOutputs;

      RigidBody elevator = fullRobotModel.getElevator();
      this.inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, -processedSensors.getGravityInWorldFrame().getZ());

      this.bipedFeet = bipedFeet;
      this.bipedSupportPolygons = bipedSupportPolygons;
      footPolygonVisualizer = new FootPolygonVisualizer(bipedFeet, dynamicGraphicObjectsListRegistry, registry);

      ReferenceFrame elevatorFrame = fullRobotModel.getElevatorFrame();
      for (RobotSide robotSide : RobotSide.values())
      {
         for (LimbName limbName : LimbName.values())
         {
            ReferenceFrame endEffectorFrame = fullRobotModel.getEndEffectorFrame(robotSide, limbName);
            desiredEndEffectorAccelerationsInWorld.get(robotSide).put(limbName,
                    new SpatialAccelerationVector(endEffectorFrame, elevatorFrame, endEffectorFrame));
         }

         spatialAccelerationProjectors.put(robotSide,
                                           new SpatialAccelerationProjector(robotSide.getCamelCaseNameForStartOfExpression()
                                              + "FootSpatialAccelerationProjector", registry));
         isCoPOnEdge.put(robotSide, new BooleanYoVariable("is" + robotSide.getCamelCaseNameForMiddleOfExpression() + "CoPOnEdge", registry));

         EndEffectorPoseTwistAndSpatialAccelerationCalculator feetPoseTwistAndSpatialAccelerationCalculator =
            new EndEffectorPoseTwistAndSpatialAccelerationCalculator(fullRobotModel.getEndEffector(robotSide, LimbName.LEG),
               fullRobotModel.getEndEffectorFrame(robotSide, LimbName.LEG), twistCalculator);
         footPoseTwistAndSpatialAccelerationCalculators.put(robotSide, feetPoseTwistAndSpatialAccelerationCalculator);
      }

      this.desiredHeadingControlModule = desiredHeadingControlModule;
      midFeetZUp = referenceFrames.getMidFeetZUpFrame();

      for (RobotSide robotSide : RobotSide.values())
      {
         bipedFeet.get(robotSide).setIsSupportingFoot(true);
      }

      bipedSupportPolygons.update(bipedFeet.get(RobotSide.LEFT), bipedFeet.get(RobotSide.RIGHT));

      SpeedControllingDesiredCoPCalculator desiredCapturePointToDesiredCoPControlModule = new SpeedControllingDesiredCoPCalculator(processedSensors,
                                                                                             referenceFrames, registry, dynamicGraphicObjectsListRegistry);
      desiredCapturePointToDesiredCoPControlModule.setParametersForR2InverseDynamics();
      SimpleDesiredCenterOfPressureFilter desiredCenterOfPressureFilter = new SimpleDesiredCenterOfPressureFilter(bipedSupportPolygons, referenceFrames,
                                                                             controlDT, registry);
      desiredCenterOfPressureFilter.setParametersForR2InverseDynamics();

      CapturabilityBasedDesiredCoPVisualizer visualizer = new CapturabilityBasedDesiredCoPVisualizer(registry, dynamicGraphicObjectsListRegistry);
      SacrificeDeltaCMPDesiredCoPAndCMPControlModule desiredCoPAndCMPControlModule =
         new SacrificeDeltaCMPDesiredCoPAndCMPControlModule(desiredCapturePointToDesiredCoPControlModule, desiredCapturePointToDesiredCoPControlModule,
            desiredCenterOfPressureFilter, visualizer, bipedSupportPolygons, processedSensors, referenceFrames, registry);
      desiredCoPAndCMPControlModule.setGains(3e-2, 1.0);
      this.desiredCoPAndCMPControlModule = desiredCoPAndCMPControlModule;


      // this.desiredCoPAndCMPControlModule = new SacrificeCMPCoPAndCMPControlModule(desiredCapturePointToDesiredCoPControlModule,
      // desiredCapturePointToDesiredCoPControlModule, desiredCenterOfPressureFilter, visualizer, bipedSupportPolygons, processedSensors, referenceFrames, registry).setGains(3e-2, 1.0);

      RigidBody pelvis = fullRobotModel.getPelvis();
      RigidBody chest = fullRobotModel.getChest();
      this.chestAngularAccelerationcalculator = new ThreeDoFAngularAccelerationControlModule(pelvis, chest);

      
      virtualToePointCalculator = new NewGeometricVirtualToePointCalculator(referenceFrames, registry, dynamicGraphicObjectsListRegistry, 0.95);

      this.legStrengthCalculator = new TeeterTotterLegStrengthCalculator(registry);

      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      this.centerOfMassHeightControlModule = new PartialCenterOfMassHeightControlModule(processedSensors, registry, highLevelHumanoidController.getCoMControlType());
      
      centerOfMassHeightControlModule.setParametersForR2InverseDynamics();
      
      this.totalMass = TotalMassCalculator.computeSubTreeMass(elevator);

      this.highLevelHumanoidController = highLevelHumanoidController;
      optimizer = new BipedMomentumOptimizer(fullRobotModel, referenceFrames.getCenterOfMassFrame(), controlDT, twistCalculator, registry);

      this.desiredPelvisLinearAcceleration = new YoFrameVector("desiredPelvisLinearAcceleration", "", referenceFrames.getPelvisFrame(), registry);
      this.desiredPelvisAngularAcceleration = new YoFrameVector("desiredPelvisAngularAcceleration", "", referenceFrames.getPelvisFrame(), registry);
      this.desiredPelvisForce = new YoFrameVector("desiredPelvisForce", "", centerOfMassFrame, registry);
      this.desiredPelvisTorque = new YoFrameVector("desiredPelvisTorque", "", centerOfMassFrame, registry);

      for (RobotSide robotSide : RobotSide.values())
      {
         String swingfootPositionName = "desired" + robotSide.getCamelCaseNameForMiddleOfExpression() + "SwingFootPositionInWorld";
         YoFramePoint desiredSwingFootPosition = new YoFramePoint(swingfootPositionName, "", worldFrame, registry);
         desiredFootPositionsInWorld.put(robotSide, desiredSwingFootPosition);    // TODO: why is this here?
         DynamicGraphicPosition desiredSwingFootPositionViz = new DynamicGraphicPosition(swingfootPositionName, desiredSwingFootPosition, 0.03,
                                                                 YoAppearance.Orange());
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(name, desiredSwingFootPositionViz);
      }

      DynamicGraphicPosition capturePointViz = capturePoint.createDynamicGraphicPosition("Capture Point", 0.01, YoAppearance.Blue(), GraphicType.ROTATED_CROSS);
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("Capture Point", capturePointViz);
      dynamicGraphicObjectsListRegistry.registerArtifact("Capture Point", capturePointViz.createArtifact());

      InverseDynamicsJoint[] joints = ScrewTools.computeJointsInOrder(elevator);
      for (InverseDynamicsJoint joint : joints)
      {
         if (joint instanceof RevoluteJoint)
         {
            desiredAccelerationYoVariables.put((RevoluteJoint) joint, new DoubleYoVariable(joint.getName() + "qdd_d", registry));
         }
      }
      
      this.doChestOrientationControl = doChestOrientationControl;

      kAngularMomentumZ.set(10.0);    // 50.0); // 10.0);
      kPelvisYaw.set(100.0);    // was 0.0 for M3 movie
      kUpperBody.set(100.0);
      zetaUpperBody.set(1.0);
      omega0.set(3.0);    // just to initialize, will be reset every tick. TODO: integrate ICP control law, fz calculation and omega0 calculation
   }

   public void initialize()
   {
      optimizer.initialize();
      highLevelHumanoidController.setCapturePoint(computeCapturePoint());
      highLevelHumanoidController.setOmega0(omega0.getDoubleValue());
      highLevelHumanoidController.initialize();
      doControl();
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return getName();
   }

   public void doControl()
   {
      desiredHeadingControlModule.updateDesiredHeadingFrame();
      FramePoint2d capturePoint = computeCapturePoint();

      for (RobotSide robotSide : RobotSide.values())
      {
         bipedFeet.get(robotSide).setIsSupportingFoot(isFootConstrained(robotSide));
      }

      BipedFootInterface leftFoot = bipedFeet.get(RobotSide.LEFT);
      BipedFootInterface rightFoot = bipedFeet.get(RobotSide.RIGHT);

      bipedSupportPolygons.update(leftFoot, rightFoot);
      footPolygonVisualizer.update();

      highLevelHumanoidController.setCapturePoint(capturePoint);
      highLevelHumanoidController.setOmega0(omega0.getDoubleValue());
      highLevelHumanoidController.doControl();

      doMomentumBasedControl(capturePoint);
      inverseDynamicsCalculator.compute();
      fullRobotModel.setTorques(processedOutputs);
      updateYoVariables(capturePoint);
   }

   private FramePoint2d computeCapturePoint()
   {
      FramePoint2d ret = processedSensors.getCenterOfMassPositionInFrame(worldFrame).toFramePoint2d();
      FrameVector2d velocityPart = processedSensors.getCenterOfMassVelocityInFrame(worldFrame).toFrameVector2d();
      velocityPart.scale(1.0 / omega0.getDoubleValue());
      ret.add(velocityPart);

      return ret;
   }

   private void doMomentumBasedControl(FramePoint2d capturePoint)
   {
      ReferenceFrame frame = worldFrame;
      RobotSide supportLeg = highLevelHumanoidController.getSupportLeg();
      FramePoint2d desiredCapturePoint = new FramePoint2d(worldFrame);
      highLevelHumanoidController.packDesiredICP(desiredCapturePoint);
      FrameVector2d desiredCapturePointVelocity = new FrameVector2d(worldFrame);
      highLevelHumanoidController.packDesiredICPVelocity(desiredCapturePointVelocity);
      double desiredPelvisRoll = highLevelHumanoidController.getDesiredPelvisRoll();
      double desiredPelvisPitch = highLevelHumanoidController.getDesiredPelvisPitch();
      desiredCoPAndCMPControlModule.compute(capturePoint, supportLeg, desiredCapturePoint, desiredCapturePointVelocity, desiredPelvisRoll, desiredPelvisPitch,
              omega0.getDoubleValue());
      FramePoint2d desiredCoP = new FramePoint2d(worldFrame);
      desiredCoPAndCMPControlModule.packCoP(desiredCoP);
      FramePoint2d desiredCMP = new FramePoint2d(worldFrame);
      desiredCoPAndCMPControlModule.packCMP(desiredCMP);
      highLevelHumanoidController.setPreviousCoP(desiredCoP);

      desiredCoP.changeFrame(frame);
      desiredCMP.changeFrame(frame);

      FrameVector2d desiredDeltaCMP = new FrameVector2d(desiredCMP);
      desiredDeltaCMP.sub(desiredCoP);

      fixDesiredCoPNumericalRoundoff(desiredCoP, bipedSupportPolygons.getSupportPolygonInMidFeetZUp());
      desiredCoP.changeFrame(frame);

      FramePoint com = processedSensors.getCenterOfMassPositionInFrame(desiredHeadingControlModule.getDesiredHeadingFrame());

      double fZ = computeFz(supportLeg, desiredCoP, com);

      SideDependentList<FramePoint2d> virtualToePoints = new SideDependentList<FramePoint2d>();
      if (supportLeg == null)
      {
         virtualToePointCalculator.packVirtualToePoints(virtualToePoints, bipedSupportPolygons, desiredCoP,
                 highLevelHumanoidController.getUpcomingSupportLeg());
      }
      else
      {
         FrameConvexPolygon2d footPolygonInAnkleZUp = bipedSupportPolygons.getFootPolygonInAnkleZUp(supportLeg);
         fixDesiredCoPNumericalRoundoff(desiredCoP, footPolygonInAnkleZUp);
         desiredCoP.changeFrame(frame);

         virtualToePoints.put(supportLeg, desiredCoP);
         virtualToePoints.put(supportLeg.getOppositeSide(), new FramePoint2d(frame));
      }

      legStrengthCalculator.packLegStrengths(lambdas, virtualToePoints, desiredCoP);

      SideDependentList<FramePoint> virtualToePointsOnSole = new SideDependentList<FramePoint>();
      for (RobotSide robotSide : RobotSide.values())
      {
         virtualToePoints.get(robotSide).changeFrame(worldFrame);
         FramePoint virtualToePoint = virtualToePoints.get(robotSide).toFramePoint();
         virtualToePoint = projectPointOntoSole(robotSide, virtualToePoint);
         virtualToePoint.changeFrame(worldFrame);
         virtualToePointsOnSole.put(robotSide, virtualToePoint);
      }

      FrameLine2d vtpToVTPLine = new FrameLine2d(virtualToePointsOnSole.get(RobotSide.LEFT).toFramePoint2d(),
                                    virtualToePointsOnSole.get(RobotSide.RIGHT).toFramePoint2d());

      FramePoint r1 = virtualToePointsOnSole.get(RobotSide.LEFT);
      FramePoint2d r12d = r1.toFramePoint2d();
      vtpToVTPLine.orthogonalProjection(r12d);    // not sure if necessary.
      double x1 = vtpToVTPLine.getParameterGivenPointEpsilon(r12d, 1e-12);
      double z1 = r1.getZ();

      FramePoint r2 = virtualToePointsOnSole.get(RobotSide.RIGHT);
      FramePoint2d r22d = r2.toFramePoint2d();
      vtpToVTPLine.orthogonalProjection(r22d);    // not sure if necessary.
      double x2 = vtpToVTPLine.getParameterGivenPointEpsilon(r22d, 1e-12);
      double z2 = r2.getZ();
      double z = com.getZ();

      double omega0Squared = (fZ * (x1 - x2))
                             / (totalMass
                                * (x1 * (z - lambdas.get(RobotSide.LEFT) * z1 + (-1 + lambdas.get(RobotSide.LEFT)) * z2)
                                   + x2 * (-z + z1 - lambdas.get(RobotSide.RIGHT) * z1 + lambdas.get(RobotSide.RIGHT) * z2)));

      if (omega0Squared <= 0.0)
         throw new RuntimeException("omega0Squared <= 0.0. omega0Squared = " + omega0Squared);

      double omega0 = Math.sqrt(omega0Squared);
      this.omega0.set(omega0);
      double k1PlusK2 = omega0Squared * totalMass;
      SideDependentList<Double> ks = new SideDependentList<Double>();
      for (RobotSide robotSide : RobotSide.values())
      {
         ks.put(robotSide, lambdas.get(robotSide) * k1PlusK2);
      }

      FrameVector totalgroundReactionMoment = determineGroundReactionMoment();
      Wrench totalGroundReactionWrench = new Wrench(centerOfMassFrame, centerOfMassFrame);

      for (RobotSide robotSide : RobotSide.values())
      {
         FramePoint groundReactionForceTerminalPoint = new FramePoint(centerOfMassFrame);
         groundReactionForceTerminalPoint.changeFrame(frame);
         desiredDeltaCMP.changeFrame(frame);
         groundReactionForceTerminalPoint.setX(groundReactionForceTerminalPoint.getX() - desiredDeltaCMP.getX());
         groundReactionForceTerminalPoint.setY(groundReactionForceTerminalPoint.getY() - desiredDeltaCMP.getY());
         FrameVector groundReactionForce = new FrameVector(groundReactionForceTerminalPoint);

         FramePoint virtualToePoint = virtualToePointsOnSole.get(robotSide);
         virtualToePoint.changeFrame(frame);
         groundReactionForce.sub(virtualToePoint);
         groundReactionForce.scale(ks.get(robotSide));

         FrameVector groundReactionMoment = new FrameVector(totalgroundReactionMoment);

         // TODO: base on contact situation.
         groundReactionMoment.scale(lambdas.get(robotSide));

         RigidBody foot = fullRobotModel.getFoot(robotSide);
         ReferenceFrame footCoMFrame = foot.getBodyFixedFrame();
         FrameVector torque = new FrameVector(frame);
         torque.cross(virtualToePoint, groundReactionForce);
         Wrench groundReactionWrench = new Wrench(footCoMFrame, frame, groundReactionForce.getVector(), torque.getVector());
         groundReactionWrench.addAngularPart(groundReactionMoment.getVector());
         groundReactionWrench.changeFrame(footCoMFrame);
         inverseDynamicsCalculator.setExternalWrench(foot, groundReactionWrench);
         groundReactionWrench.changeFrame(centerOfMassFrame);
         groundReactionWrench.changeBodyFrameAttachedToSameBody(centerOfMassFrame);
         totalGroundReactionWrench.add(groundReactionWrench);
      }

      double kUpperBody = this.kUpperBody.getDoubleValue();
      double dUpperBody = 2.0 * zetaUpperBody.getDoubleValue() * Math.sqrt(kUpperBody);
      for (RobotSide robotSide : RobotSide.values())
      {
         doPDControl(kUpperBody, dUpperBody, fullRobotModel.getArmJointList(robotSide));
      }

      doPDControl(kUpperBody, dUpperBody, fullRobotModel.getNeckJointList());

      if (doChestOrientationControl)
      {
         chestAngularAccelerationcalculator.doControl(highLevelHumanoidController.getChestAngularAcceleration());         
      }
      else
      {         
         doPDControl(kUpperBody, dUpperBody, fullRobotModel.getSpineJointList());
      }

      FrameVector desiredAngularCentroidalMomentumRate = new FrameVector(totalGroundReactionWrench.getExpressedInFrame(),
                                                            totalGroundReactionWrench.getAngularPartCopy());
      FrameVector desiredLinearCentroidalMomentumRate = new FrameVector(totalGroundReactionWrench.getExpressedInFrame(),
                                                           totalGroundReactionWrench.getLinearPartCopy());
      desiredLinearCentroidalMomentumRate.setZ(desiredLinearCentroidalMomentumRate.getZ() + processedSensors.getGravityInWorldFrame().getZ() * totalMass);

      for (RobotSide robotSide : RobotSide.values())
      {
         boolean isSwingLeg = supportLeg == robotSide.getOppositeSide();
         double maxKneeAngle = 0.4;
         boolean leavingKneeLockRegion = optimizer.leavingSingularRegion(robotSide, LimbName.LEG)
                                         && (fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE).getQ() < maxKneeAngle);    // TODO: hack
         boolean trajectoryInitialized = highLevelHumanoidController.trajectoryInitialized(robotSide);

         BooleanYoVariable inSingularRegion = inSingularRegions.get(robotSide);
         inSingularRegion.set(optimizer.inSingularRegion(robotSide, LimbName.LEG));

         // if ((supportLeg == robotSide.getOppositeSide()) &&!optimizer.inSingularRegion(robotSide) &&!stateMachine.trajectoryInitialized(robotSide))
         if (isSwingLeg && (leavingKneeLockRegion || (!inSingularRegion.getBooleanValue() &&!trajectoryInitialized)))
         {
            SpatialAccelerationVector taskSpaceAcceleration = new SpatialAccelerationVector();
            optimizer.computeMatchingNondegenerateTaskSpaceAcceleration(robotSide, LimbName.LEG, taskSpaceAcceleration);
            highLevelHumanoidController.initializeTrajectory(robotSide, taskSpaceAcceleration);
         }

         for (LimbName limbName : LimbName.values())
         {
            SpatialAccelerationVector desiredEndEffectorAccelerationInWorld = desiredEndEffectorAccelerationsInWorld.get(robotSide).get(limbName);
            desiredEndEffectorAccelerationInWorld.set(highLevelHumanoidController.getEndEffectorAcceleration(robotSide, limbName));

            if (limbName == LimbName.LEG)
            {
               BipedFootInterface bipedFoot = bipedFeet.get(robotSide);
               FrameConvexPolygon2d footPolygon = bipedFoot.getFootPolygonInSoleFrame();
               FramePoint footCoPOnSole = virtualToePointsOnSole.get(robotSide);
               footCoPOnSole.changeFrame(footPolygon.getReferenceFrame());
               FramePoint2d footCoPOnSole2d = footCoPOnSole.toFramePoint2d();
               FrameLineSegment2d closestEdge = footPolygon.getClosestEdge(footCoPOnSole2d);
               double epsilonPointOnEdge = 1e-3;
               boolean isCoPOnEdge = closestEdge.distance(footCoPOnSole2d) < epsilonPointOnEdge;
               this.isCoPOnEdge.get(robotSide).set(isCoPOnEdge);

               boolean isConstrained = isFootConstrained(robotSide);

               if (isConstrained)
               {
                  if (isCoPOnEdge)
                  {
                     spatialAccelerationProjectors.get(robotSide).projectAcceleration(desiredEndEffectorAccelerationInWorld, closestEdge);
                  }
                  else
                  {
                     // use zero angular acceleration and zero linear acceleration of origin
                     desiredEndEffectorAccelerationInWorld.set(
                         footPoseTwistAndSpatialAccelerationCalculators.get(robotSide).calculateDesiredEndEffectorSpatialAccelerationFromDesiredAccelerations(
                            new FrameVector(worldFrame), new FrameVector(worldFrame), fullRobotModel.getElevator()));
                  }

               }
            }

            optimizer.setControlMode(robotSide, limbName, highLevelHumanoidController.getControlMode(robotSide, limbName));
            optimizer.setDesiredEndEffectorAccelerationInWorld(robotSide, limbName, desiredEndEffectorAccelerationInWorld);
            optimizer.setNullspaceMultiplier(robotSide, limbName, highLevelHumanoidController.getNullspaceMultiplier(robotSide, limbName));
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         Wrench handWrench = highLevelHumanoidController.getExternalHandWrench(robotSide);
         inverseDynamicsCalculator.setExternalWrench(fullRobotModel.getHand(robotSide), handWrench);
      }

     // Use the current values as the initial guess for the next optimization.
      double[] initialGuess = new double[]
            {
            0.0,
            desiredPelvisAngularAcceleration.getX(),
            desiredPelvisAngularAcceleration.getY(),
            desiredPelvisAngularAcceleration.getZ(),
            desiredPelvisLinearAcceleration.getX(),
            desiredPelvisLinearAcceleration.getY(),
            desiredPelvisLinearAcceleration.getZ(),
            };


      optimizer.solveForRootJointAcceleration(initialGuess, desiredAngularCentroidalMomentumRate, desiredLinearCentroidalMomentumRate);
   }

   private void fixDesiredCoPNumericalRoundoff(FramePoint2d desiredCoP, FrameConvexPolygon2d polygon)
   {
      double epsilon = 1e-10;
      desiredCoP.changeFrame(polygon.getReferenceFrame());
      FramePoint2d originalDesiredCoP = new FramePoint2d(desiredCoP);
      polygon.orthogonalProjection(desiredCoP);
      double distance = originalDesiredCoP.distance(desiredCoP);
      if (distance > epsilon)
         throw new RuntimeException("desired CoP outside polygon by " + distance);
   }

   private double computeFz(RobotSide supportLeg, FramePoint2d desiredCoP, FramePoint com)
   {
      double dzdxDesired = highLevelHumanoidController.getDesiredCoMHeightSlope();
      double d2zdx2Desired = highLevelHumanoidController.getDesiredCoMHeightSecondDerivative();
      ReferenceFrame desiredHeadingFrame = desiredHeadingControlModule.getDesiredHeadingFrame();
      FrameVector comd = processedSensors.getCenterOfMassVelocityInFrame(desiredHeadingFrame);
      double xd = comd.getX();
      double copX = desiredCoP.changeFrameCopy(desiredHeadingFrame).getX();
      double xdd = MathTools.square(omega0.getDoubleValue()) * (com.getX() - copX);    // TODO: use current omega0 instead of previous

      double zDesired = highLevelHumanoidController.getDesiredCoMHeight();
      double zdDesired = dzdxDesired * xd;
      double zddDesired = d2zdx2Desired * MathTools.square(xd) + dzdxDesired * xdd;

      double fZ = centerOfMassHeightControlModule.doCenterOfMassHeightControl(zDesired, zdDesired, zddDesired, supportLeg, highLevelHumanoidController.getCoMControlType());

      return fZ;
   }

   private FramePoint projectPointOntoSole(RobotSide robotSide, FramePoint virtualToePoint)
   {
      ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
      FramePoint pointOnPlane = new FramePoint(soleFrame);
      FrameVector planeNormal = new FrameVector(soleFrame, 0.0, 0.0, 1.0);
      FramePoint lineStart = virtualToePoint.changeFrameCopy(soleFrame);
      FramePoint lineEnd = new FramePoint(virtualToePoint);    // start at VTP
      lineEnd.setZ(lineEnd.getZ() - 1.0);    // down an arbitrary amount in the frame in which the VTP is expressed
      lineEnd.changeFrame(soleFrame);    // then change frame to sole frame

      return GeometryTools.getIntersectionBetweenLineAndPlane(pointOnPlane, planeNormal, lineStart, lineEnd);
   }

   private FrameVector determineGroundReactionMoment()
   {
      FrameVector ret = new FrameVector(midFeetZUp);
      FrameVector angularMomentum = processedSensors.getAngularMomentumInFrame(midFeetZUp);
      Matrix3d pelvisToWorld = new Matrix3d();
      fullRobotModel.getPelvis().getBodyFixedFrame().getTransformToDesiredFrame(desiredHeadingControlModule.getDesiredHeadingFrame()).get(pelvisToWorld);    // TODO: take into account the twist of the desired heading frame w.r.t world.
      double pelvisYaw = RotationFunctions.getYaw(pelvisToWorld);
      ret.setZ(-kAngularMomentumZ.getDoubleValue() * angularMomentum.getZ() - kPelvisYaw.getDoubleValue() * pelvisYaw);

      return ret;
   }

   private void updateYoVariables(FramePoint2d capturePoint2d)
   {
      SpatialAccelerationVector pelvisAcceleration = new SpatialAccelerationVector();
      fullRobotModel.getRootJoint().packDesiredJointAcceleration(pelvisAcceleration);
      desiredPelvisLinearAcceleration.set(pelvisAcceleration.getLinearPartCopy());
      desiredPelvisAngularAcceleration.set(pelvisAcceleration.getAngularPartCopy());

      Wrench pelvisJointWrench = new Wrench();
      fullRobotModel.getRootJoint().packWrench(pelvisJointWrench);
      pelvisJointWrench.changeFrame(centerOfMassFrame);
      desiredPelvisForce.set(pelvisJointWrench.getLinearPartCopy());
      desiredPelvisTorque.set(pelvisJointWrench.getAngularPartCopy());

      for (RevoluteJoint joint : desiredAccelerationYoVariables.keySet())
      {
         desiredAccelerationYoVariables.get(joint).set(joint.getQddDesired());
      }

      FramePoint capturePoint = capturePoint2d.toFramePoint();
      capturePoint.changeFrame(worldFrame);
      this.capturePoint.set(capturePoint);
   }

   private static void doPDControl(double k, double d, RevoluteJoint[] joints)
   {
      for (RevoluteJoint joint : joints)
      {
         joint.setQddDesired(computeDesiredAcceleration(k, d, 0.0, 0.0, joint));
      }
   }

   private static double computeDesiredAcceleration(double k, double d, double qDesired, double qdDesired, RevoluteJoint joint)
   {
      return k * (qDesired - joint.getQ()) + d * (qdDesired - joint.getQd());
   }

   private boolean isFootConstrained(RobotSide robotSide)
   {
      RobotSide supportLeg = highLevelHumanoidController.getSupportLeg();
      if (supportLeg == null)
         return true;
      else
         return robotSide == supportLeg;
   }
}
