package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.HashMap;

import javax.media.j3d.Transform3D;
import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedFootInterface;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.FootPolygonVisualizer;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ResizableBipedFoot;
import us.ihmc.commonWalkingControlModules.captureRegion.CapturePointCalculatorInterface;
import us.ihmc.commonWalkingControlModules.captureRegion.CommonCapturePointCalculator;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DesiredCapturePointToDesiredCoPControlModule;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.LegStrengthCalculator;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.VirtualToePointCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.CenterOfMassHeightControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.NewGeometricVirtualToePointCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.TeeterTotterLegStrengthCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.pelvisOrientation.AxisAnglePelvisOrientationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP.CapturabilityBasedDesiredCoPVisualizer;
import us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP.SimpleDesiredCenterOfPressureFilter;
import us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP.SpeedControllingDesiredCoPCalculator;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.outputs.ProcessedOutputsInterface;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
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
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoAppearance;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint2d;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector2d;

public class MomentumBasedController implements RobotController
{
   private static final long serialVersionUID = -744968203951905486L;
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final ProcessedSensorsInterface processedSensors;

   private final ProcessedOutputsInterface processedOutputs;
   private final InverseDynamicsCalculator inverseDynamicsCalculator;

   private final AxisAnglePelvisOrientationControlModule orientationControlModule;
   private final BipedSupportPolygons bipedSupportPolygons;
   private final CapturePointCalculatorInterface capturePointCalculator;
   private final VirtualToePointCalculator virtualToePointCalculator;
   private final DesiredCapturePointToDesiredCoPControlModule desiredCapturePointToDesiredCoPControlModule;
   private final SimpleDesiredCenterOfPressureFilter desiredCenterOfPressureFilter;
   private final CenterOfMassHeightControlModule centerOfMassHeightControlModule;
   private final LegStrengthCalculator legStrengthCalculator;

   private final SideDependentList<BipedFootInterface> bipedFeet = new SideDependentList<BipedFootInterface>();
   private final ReferenceFrame midFeetZUp;
   private final double totalMass;

   private final MomentumBasedControllerStateMachine stateMachine;
   private final BipedMomentumOptimizer optimizer;

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final FullRobotModel fullRobotModel;
   private final CommonWalkingReferenceFrames referenceFrames;

   private final SideDependentList<SpatialAccelerationVector> desiredFootAccelerationsInWorld = new SideDependentList<SpatialAccelerationVector>();
   private final YoFrameVector swingFootPositionErrorInWorld = new YoFrameVector("swingFootPositionErrorInWorld", "", worldFrame, registry);
   private final SideDependentList<YoFramePoint> desiredFootPositionsInWorld = new SideDependentList<YoFramePoint>();
   private final SideDependentList<FootSpatialAccelerationControlModule> footSpatialAccelerationControlModules =
      new SideDependentList<FootSpatialAccelerationControlModule>();

   private final YoFrameVector desiredPelvisLinearAcceleration;
   private final YoFrameVector desiredPelvisAngularAcceleration;
   private final YoFrameVector desiredPelvisForce;
   private final YoFrameVector desiredPelvisTorque;
   private final ReferenceFrame centerOfMassFrame;

   private final DoubleYoVariable kAngularMomentumXY = new DoubleYoVariable("kAngularMomentumXY", registry);
   private final DoubleYoVariable kPelvisAxisAngle = new DoubleYoVariable("kPelvisAxisAngle", registry);
   private final DoubleYoVariable kUpperBody = new DoubleYoVariable("kUpperBody", registry);
   private final DoubleYoVariable zetaUpperBody = new DoubleYoVariable("zetaUpperBody", registry);
   private final DoubleYoVariable kAngularMomentumZ = new DoubleYoVariable("kAngularMomentumZ", registry);
   private final DoubleYoVariable kPelvisYaw = new DoubleYoVariable("kPelvisZ", registry);
   private final HashMap<RevoluteJoint, DoubleYoVariable> desiredAccelerationYoVariables = new HashMap<RevoluteJoint, DoubleYoVariable>();

   // TODO: move to separate class that takes care of determining desired GRFs
   private final YoFrameVector2d desiredDeltaCMP = new YoFrameVector2d("desiredDeltaCMP", "", worldFrame, registry);
   private final YoFramePoint2d desiredCoP = new YoFramePoint2d("desiredCoP", "", worldFrame, registry);
   private final SideDependentList<Double> legStrengths = new SideDependentList<Double>();

   private final CapturabilityBasedDesiredCoPVisualizer visualizer;
   private final FootPolygonVisualizer footPolygonVisualizer;



   public MomentumBasedController(ProcessedSensorsInterface processedSensors, ProcessedOutputsInterface processedOutputs,
                                  CommonWalkingReferenceFrames referenceFrames, TwistCalculator twistCalculator, double controlDT,
                                  DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, double footForward, double footBack, double footWidth,
                                  double footHeight, SideDependentList<FootSwitchInterface> footSwitches)
   {
      this.processedSensors = processedSensors;
      this.fullRobotModel = processedSensors.getFullRobotModel();
      this.referenceFrames = referenceFrames;
      this.processedOutputs = processedOutputs;
      this.visualizer = new CapturabilityBasedDesiredCoPVisualizer(registry, dynamicGraphicObjectsListRegistry);

      RigidBody elevator = fullRobotModel.getElevator();
      this.inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, -processedSensors.getGravityInWorldFrame().getZ());
      
//    ContactBasedBipedFeetUpdater bipedFeetUpdater = new ContactBasedBipedFeetUpdater(processedSensors, referenceFrames);
//    this.bipedFeetUpdater = bipedFeetUpdater;
      double footRotationPreventionFactor = 1.0;    // 0.95;

//    for (RobotSide robotSide : RobotSide.values())
      {
//       bipedFeet.put(robotSide, new SimpleBipedFoot(referenceFrames, robotSide, footRotationPreventionFactor * footForward, footRotationPreventionFactor * footBack, footRotationPreventionFactor * footWidth / 2.0, footRotationPreventionFactor * footWidth / 2.0, registry));
//       HashMap<FramePoint2d, Boolean> contactMap = processedSensors.getContactMap(robotSide);
//       bipedFeet.put(robotSide, new ContactBasedBipedFoot(referenceFrames, robotSide, new FrameConvexPolygon2d(contactMap.keySet()), registry));
      }
//      BipedFootInterface leftFoot = bipedFeet.get(RobotSide.LEFT);
//      BipedFootInterface rightFoot = bipedFeet.get(RobotSide.RIGHT);
      
      double narrowWidthOnToesPercentage = 1.0;
      double maxToePointsBack = 0.999;
      double maxHeelPointsForward = 0.999;
      ResizableBipedFoot rightFoot = ResizableBipedFoot.createRectangularRightFoot(footRotationPreventionFactor, footRotationPreventionFactor, footForward, footBack, footWidth,
           footHeight, narrowWidthOnToesPercentage, maxToePointsBack, maxHeelPointsForward , referenceFrames, registry, dynamicGraphicObjectsListRegistry);
      ResizableBipedFoot leftFoot = rightFoot.createLeftFootAsMirrorImage(referenceFrames, registry, dynamicGraphicObjectsListRegistry);
      bipedFeet.put(RobotSide.RIGHT, rightFoot);
      bipedFeet.put(RobotSide.LEFT, leftFoot);

      footPolygonVisualizer = new FootPolygonVisualizer(bipedFeet, dynamicGraphicObjectsListRegistry, registry);

      ReferenceFrame elevatorFrame = fullRobotModel.getElevatorFrame();
      for (RobotSide robotSide : RobotSide.values())
      {
         ReferenceFrame footFrame = referenceFrames.getFootFrame(robotSide);
         desiredFootAccelerationsInWorld.put(robotSide, new SpatialAccelerationVector(footFrame, elevatorFrame, footFrame));
         footSpatialAccelerationControlModules.put(robotSide,
                 new FootSpatialAccelerationControlModule(referenceFrames, twistCalculator, bipedFeet.get(robotSide), fullRobotModel, registry));
      }

      this.orientationControlModule = new AxisAnglePelvisOrientationControlModule(processedSensors, referenceFrames, null, registry, false);

      SideDependentList<ReferenceFrame> ankleZUpFrames = referenceFrames.getAnkleZUpReferenceFrames();
      midFeetZUp = referenceFrames.getMidFeetZUpFrame();
      this.bipedSupportPolygons = new BipedSupportPolygons(ankleZUpFrames, midFeetZUp, registry, dynamicGraphicObjectsListRegistry);


      this.capturePointCalculator = new CommonCapturePointCalculator(processedSensors, referenceFrames, true, registry, dynamicGraphicObjectsListRegistry);

      for (RobotSide robotSide : RobotSide.values())
      {
         bipedFeet.get(robotSide).setIsSupportingFoot(true);
      }
      bipedSupportPolygons.update(leftFoot, rightFoot);

      double maximumLegStrengthWhenTransferringAway = 0.9;
      this.virtualToePointCalculator = new NewGeometricVirtualToePointCalculator(referenceFrames, registry, dynamicGraphicObjectsListRegistry,
              maximumLegStrengthWhenTransferringAway);
      
//      this.virtualToePointCalculator = new GeometricVirtualToePointCalculator(referenceFrames, registry, dynamicGraphicObjectsListRegistry);
//      virtualToePointCalculator.setNewR2Parameters();
      
      SpeedControllingDesiredCoPCalculator desiredCapturePointToDesiredCoPControlModule = new SpeedControllingDesiredCoPCalculator(processedSensors,
                                                                                             referenceFrames, registry, dynamicGraphicObjectsListRegistry);
      desiredCapturePointToDesiredCoPControlModule.setParametersForR2InverseDynamics();
      this.desiredCapturePointToDesiredCoPControlModule = desiredCapturePointToDesiredCoPControlModule;
      desiredCenterOfPressureFilter = new SimpleDesiredCenterOfPressureFilter(bipedSupportPolygons, referenceFrames, controlDT, registry);
      desiredCenterOfPressureFilter.setParametersForR2InverseDynamics();
      this.legStrengthCalculator = new TeeterTotterLegStrengthCalculator(registry);

      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      this.centerOfMassHeightControlModule = new CenterOfMassHeightControlModule(processedSensors, registry);
      centerOfMassHeightControlModule.setParametersForR2InverseDynamics();
      this.totalMass = TotalMassCalculator.computeSubTreeMass(elevator);
      orientationControlModule.setupParametersForR2();

      stateMachine = new MomentumBasedControllerStateMachine(fullRobotModel, referenceFrames, twistCalculator, bipedFeet, bipedSupportPolygons, footSwitches,
              processedSensors, capturePointCalculator, processedSensors.getYoTime(), controlDT, footHeight, registry, dynamicGraphicObjectsListRegistry);
      optimizer = new BipedMomentumOptimizer(fullRobotModel, referenceFrames.getCenterOfMassFrame(), controlDT, twistCalculator, registry);

      this.desiredPelvisLinearAcceleration = new YoFrameVector("desiredPelvisLinearAcceleration", "", referenceFrames.getPelvisFrame(), registry);
      this.desiredPelvisAngularAcceleration = new YoFrameVector("desiredPelvisAngularAcceleration", "", referenceFrames.getPelvisFrame(), registry);
      this.desiredPelvisForce = new YoFrameVector("desiredPelvisForce", "", centerOfMassFrame, registry);
      this.desiredPelvisTorque = new YoFrameVector("desiredPelvisTorque", "", centerOfMassFrame, registry);

      for (RobotSide robotSide : RobotSide.values())
      {
         String swingfootPositionName = "desired" + robotSide.getCamelCaseNameForMiddleOfExpression() + "SwingFootPositionInWorld";
         YoFramePoint desiredSwingFootPosition = new YoFramePoint(swingfootPositionName, "", worldFrame, registry);
         desiredFootPositionsInWorld.put(robotSide, desiredSwingFootPosition);
         DynamicGraphicPosition desiredSwingFootPositionViz = new DynamicGraphicPosition(swingfootPositionName, desiredSwingFootPosition, 0.03,
                                                                 YoAppearance.Orange());
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(name, desiredSwingFootPositionViz);
      }

      InverseDynamicsJoint[] joints = ScrewTools.computeJointsInOrder(elevator);
      for (InverseDynamicsJoint joint : joints)
      {
         if (joint instanceof RevoluteJoint)
         {
            desiredAccelerationYoVariables.put((RevoluteJoint) joint, new DoubleYoVariable(joint.getName() + "qdd_d", registry));
         }
      }

      kAngularMomentumXY.set(3e-2);
      kPelvisAxisAngle.set(1.0); // was 1.0 for M3 video
      kAngularMomentumZ.set(10.0); // 50.0); // 10.0);
      kPelvisYaw.set(20.0); // 100.0); // was 0.0 for M3 movie
      kUpperBody.set(100.0);
      zetaUpperBody.set(1.0);
   }

   public void initialize()
   {
      optimizer.initialize();
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
      capturePointCalculator.computeCapturePoint(stateMachine.getPlantedLeg());
      for (RobotSide robotSide : RobotSide.values())
      {
         bipedFeet.get(robotSide).setIsSupportingFoot(!stateMachine.isSwingFoot(robotSide));
      }
      BipedFootInterface leftFoot = bipedFeet.get(RobotSide.LEFT);
      BipedFootInterface rightFoot = bipedFeet.get(RobotSide.RIGHT);
      
      bipedSupportPolygons.update(leftFoot, rightFoot);
      footPolygonVisualizer.update();

      ReferenceFrame frame;
      if (stateMachine.getSupportLeg() == null)
         frame = referenceFrames.getMidFeetZUpFrame();
      else
         frame = referenceFrames.getAnkleZUpFrame(stateMachine.getSupportLeg());

      double comHeight = processedSensors.getCenterOfMassPositionInFrame(frame).getZ();

      stateMachine.setPreviousCoP(desiredCoP.getFramePoint2dCopy());
      stateMachine.setCoMHeight(comHeight);
      stateMachine.checkTransitionConditionsThoroughly();
      stateMachine.doAction();

      doMomentumBasedControl();
      inverseDynamicsCalculator.compute();
      fullRobotModel.setTorques(processedOutputs);
      updateYoVariables();
   }

   private void doMomentumBasedControl()
   {
      // TODO: calculate CMP first, need to make sure that CoP is inside BoS, CMP may be outside
      FramePoint2d desiredCoP = determineDesiredCoP();
      this.desiredCoP.set(desiredCoP.changeFrameCopy(this.desiredCoP.getReferenceFrame()));
      FrameVector2d desiredDeltaCMP = determineDesiredDeltaCMP();
      FramePoint2d desiredCMP = new FramePoint2d(desiredCoP);
      desiredCMP.add(desiredDeltaCMP);
      
      if (!bipedSupportPolygons.getSupportPolygonInMidFeetZUp().isPointInside(desiredCoP.changeFrameCopy(midFeetZUp)))
         throw new RuntimeException("desired CoP outside support polygon");

      desiredDeltaCMP.changeFrame(this.desiredDeltaCMP.getReferenceFrame());
      this.desiredDeltaCMP.set(desiredDeltaCMP);

      SideDependentList<FramePoint2d> virtualToePoints = new SideDependentList<FramePoint2d>();
      RobotSide supportLeg = stateMachine.getSupportLeg();
      ReferenceFrame frame;
      if (supportLeg == null)
      {
         frame = worldFrame; // midFeetZUp;
         virtualToePointCalculator.packVirtualToePoints(virtualToePoints, bipedSupportPolygons, desiredCoP, stateMachine.getUpcomingSupportLeg());
      }
      else
      {
         FrameConvexPolygon2d footPolygonInAnkleZUp = bipedSupportPolygons.getFootPolygonInAnkleZUp(supportLeg);
         if (!footPolygonInAnkleZUp.isPointInside(desiredCoP.changeFrameCopy(footPolygonInAnkleZUp.getReferenceFrame())))
            throw new RuntimeException("desired CoP outside foot polygon");
         
         frame = worldFrame; // referenceFrames.getAnkleZUpFrame(supportLeg);
         virtualToePoints.put(supportLeg, desiredCoP);
         virtualToePoints.put(supportLeg.getOppositeSide(), new FramePoint2d(frame));
      }

      legStrengthCalculator.packLegStrengths(legStrengths, virtualToePoints, desiredCoP);

      double fZ = centerOfMassHeightControlModule.doCenterOfMassHeightControl(stateMachine.getDesiredCoMHeight(), stateMachine.getDesiredCoMHeightVelocity(),
                     stateMachine.getDesiredCoMHeightAcceleration(), supportLeg);
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
         FramePoint virtualToePoint = virtualToePoints.get(robotSide).toFramePoint();
         
         virtualToePoint = projectPointOntoSole(robotSide, virtualToePoint);
         
         virtualToePoint.changeFrame(frame);
         groundReactionForce.sub(virtualToePoint);
         groundReactionForce.scale(legStrengths.get(robotSide) * fZ / groundReactionForce.getZ());

         FrameVector groundReactionMoment = new FrameVector(totalgroundReactionMoment);
         
         // TODO: base on contact situation.
         groundReactionMoment.scale(legStrengths.get(robotSide));
//         groundReactionMoment.scale(robotSide == RobotSide.LEFT ? 1.0 : 0.0);

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

//    doChestOrientationControl();
      doPDControl(kUpperBody, dUpperBody, fullRobotModel.getSpineJointList());


      FrameVector desiredAngularCentroidalMomentumRate = new FrameVector(totalGroundReactionWrench.getExpressedInFrame(),
                                                            totalGroundReactionWrench.getAngularPartCopy());
      FrameVector desiredLinearCentroidalMomentumRate = new FrameVector(totalGroundReactionWrench.getExpressedInFrame(),
                                                           totalGroundReactionWrench.getLinearPartCopy());
      desiredLinearCentroidalMomentumRate.setZ(desiredLinearCentroidalMomentumRate.getZ() + processedSensors.getGravityInWorldFrame().getZ() * totalMass);


      for (RobotSide robotSide : RobotSide.values())
      {
//       if (supportLeg == robotSide.getOppositeSide() && optimizer.leavingSingularRegion(robotSide) && fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE).getQ() < 0.4 || (!optimizer.inSingularRegion(robotSide)) && !stateMachine.trajectoryInitialized(robotSide)) // TODO: knee angle hack
         if ((supportLeg == robotSide.getOppositeSide()) &&!optimizer.inSingularRegion(robotSide) &&!stateMachine.trajectoryInitialized(robotSide))
         {
            SpatialAccelerationVector taskSpaceAcceleration = new SpatialAccelerationVector();
            optimizer.computeMatchingNondegenerateTaskSpaceAcceleration(robotSide, taskSpaceAcceleration);
            stateMachine.initializeTrajectory(robotSide, taskSpaceAcceleration);
         }

         FramePose desiredFootPose = stateMachine.getDesiredFootPose(robotSide);
         Twist desiredFootTwist = stateMachine.getDesiredFootTwist(robotSide);
         SpatialAccelerationVector feedForwardFootSpatialAcceleration = stateMachine.getDesiredFootAcceleration(robotSide);
         boolean isSwingFoot = stateMachine.isSwingFoot(robotSide);
         footSpatialAccelerationControlModules.get(robotSide).compute(virtualToePoints.get(robotSide), desiredFootPose, desiredFootTwist,
                 feedForwardFootSpatialAcceleration, isSwingFoot);
         footSpatialAccelerationControlModules.get(robotSide).packFootAcceleration(desiredFootAccelerationsInWorld.get(robotSide));

         if (isSwingFoot)
            swingFootPositionErrorInWorld.set(footSpatialAccelerationControlModules.get(robotSide).getSwingFootPositionErrorInWorld());

         desiredFootPositionsInWorld.get(robotSide).set(desiredFootPose.getPositionInFrame(worldFrame));
      }

      optimizer.setDesiredFootAccelerationsInWorld(desiredFootAccelerationsInWorld);
      
      for (RobotSide robotSide : RobotSide.values())
      {
         optimizer.setNullspaceMultiplier(robotSide, stateMachine.getNullspaceMultiplier(robotSide));
      }

      optimizer.solveForRootJointAcceleration(desiredAngularCentroidalMomentumRate, desiredLinearCentroidalMomentumRate);
   }
   
   private FramePoint projectPointOntoSole(RobotSide robotSide, FramePoint virtualToePoint)
   {
      ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
      FramePoint pointOnPlane = new FramePoint(soleFrame);
      FrameVector planeNormal = new FrameVector(soleFrame, 0.0, 0.0, 1.0);
      FramePoint lineStart = virtualToePoint.changeFrameCopy(soleFrame);
      FramePoint lineEnd = new FramePoint(virtualToePoint); // start at VTP
      lineEnd.setZ(lineEnd.getZ() - 1.0); // down an arbitrary amount in the frame in which the VTP is expressed
      lineEnd.changeFrame(soleFrame); // then change frame to sole frame

      return GeometryTools.getIntersectionBetweenLineAndPlane(pointOnPlane, planeNormal, lineStart, lineEnd);
   }

// private void doChestOrientationControl()
// {
//    // FIXME: Complete hack.
//    RevoluteJoint[] spineJoints = fullRobotModel.getSpineJointList();
//    Transform3D chestToPelvis = spineJoints[0].getFrameAfterJoint().getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
//    double[] yawPitchRoll = new double[3];
//    RotationFunctions.getYawPitchRoll(yawPitchRoll, chestToPelvis);
//    double k = 100.0;
//    double b = 40.0;
//    spineJoints[0].setQddDesired(-k * yawPitchRoll[1] - b * spineJoints[0].getQd());
//    spineJoints[1].setQddDesired(-k * yawPitchRoll[0] - b * spineJoints[1].getQd());
//    spineJoints[2].setQddDesired(-k * yawPitchRoll[2] - b * spineJoints[2].getQd());
// }

   private FramePoint2d determineDesiredCoP()
   {
      RobotSide supportLeg = stateMachine.getSupportLeg();
      ReferenceFrame frame = (supportLeg == null) ? midFeetZUp : referenceFrames.getAnkleZUpFrame(supportLeg);
      FramePoint2d desiredCapturePoint = new FramePoint2d(frame);
      stateMachine.packDesiredICP(desiredCapturePoint);

      FrameVector2d desiredCapturePointVelocity = new FrameVector2d(frame);
      stateMachine.packDesiredICPVelocity(desiredCapturePointVelocity);

      FrameVector2d desiredVelocity = new FrameVector2d(frame);
      desiredCapturePoint.changeFrame(frame);
      FramePoint2d capturePoint = capturePointCalculator.getCapturePoint2dInFrame(frame);

      FramePoint2d desiredCoP;
      if (supportLeg == null)
      {
         desiredCoP = desiredCapturePointToDesiredCoPControlModule.computeDesiredCoPDoubleSupport(bipedSupportPolygons, capturePoint, desiredVelocity,
                 desiredCapturePoint, desiredCapturePointVelocity);
      }
      else
      {
         desiredCoP = desiredCapturePointToDesiredCoPControlModule.computeDesiredCoPSingleSupport(supportLeg, bipedSupportPolygons, capturePoint,
                 desiredVelocity, desiredCapturePoint, desiredCapturePointVelocity);
      }

      FramePoint2d filteredDesiredCoP = desiredCenterOfPressureFilter.filter(desiredCoP, supportLeg);

//    FramePoint2d filteredDesiredCoP = desiredCoP;

      visualizer.setDesiredCapturePoint(desiredCapturePoint);
      visualizer.setDesiredCoP(filteredDesiredCoP);

      return filteredDesiredCoP;
   }

   private FrameVector2d determineDesiredDeltaCMP()
   {
      RobotSide supportLeg = stateMachine.getSupportLeg();
      ReferenceFrame frame = (supportLeg == null) ? midFeetZUp : referenceFrames.getAnkleZUpFrame(supportLeg);

      FrameVector zUnitVector = new FrameVector(frame, 0.0, 0.0, 1.0);
      FrameVector angularMomentum = processedSensors.getAngularMomentumInFrame(frame);
      FrameVector desiredDeltaCMP = new FrameVector(frame);
      desiredDeltaCMP.cross(angularMomentum, zUnitVector);
      desiredDeltaCMP.scale(-kAngularMomentumXY.getDoubleValue());

      Transform3D pelvisToMidFeetZUp = fullRobotModel.getPelvis().getBodyFixedFrame().getTransformToDesiredFrame(frame);
      Matrix3d pelvisToMidFeetZUpRotation = new Matrix3d();
      pelvisToMidFeetZUp.get(pelvisToMidFeetZUpRotation);
      AxisAngle4d pelvisToCenterOfMassAxisAngle = new AxisAngle4d();
      pelvisToCenterOfMassAxisAngle.set(pelvisToMidFeetZUpRotation);
      FrameVector proportionalPart = new FrameVector(frame, pelvisToCenterOfMassAxisAngle.getX(), pelvisToCenterOfMassAxisAngle.getY(), 0.0);
      proportionalPart.scale(pelvisToCenterOfMassAxisAngle.getAngle());
      proportionalPart.cross(proportionalPart, zUnitVector);
      proportionalPart.scale(-kPelvisAxisAngle.getDoubleValue());
      desiredDeltaCMP.add(proportionalPart);

      return desiredDeltaCMP.toFrameVector2d();
   }

   private FrameVector determineGroundReactionMoment()
   {
      FrameVector ret = new FrameVector(midFeetZUp);
      FrameVector angularMomentum = processedSensors.getAngularMomentumInFrame(midFeetZUp);
      Matrix3d pelvisToWorld = new Matrix3d();
      fullRobotModel.getPelvis().getBodyFixedFrame().getTransformToDesiredFrame(ReferenceFrame.getWorldFrame()).get(pelvisToWorld);
      double pelvisYaw = RotationFunctions.getYaw(pelvisToWorld);
      ret.setZ(-kAngularMomentumZ.getDoubleValue() * angularMomentum.getZ() - kPelvisYaw.getDoubleValue() * pelvisYaw);

      return ret;
   }

   private void updateYoVariables()
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
   }

   private static void doPDControl(double k, double d, RevoluteJoint[] joints)
   {
      for (RevoluteJoint joint : joints)
      {
         joint.setQddDesired(computeDesiredAcceleration(k, d, joint));
      }
   }

   private static double computeDesiredAcceleration(double k, double d, RevoluteJoint joint)
   {
      return -k * joint.getQ() - d * joint.getQd();
   }
}
