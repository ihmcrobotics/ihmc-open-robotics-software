package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KSTStreamingState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.SensitivityBasedStabilityGradientCalculator;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.StabilityMarginRegionCalculator;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.WholeBodyContactState;
import us.ihmc.communication.PostureOptimizerState;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.PoseReferenceFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.mecano.algorithms.CentroidalMomentumCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class StabilityBasedKinematicRetargetingCalculator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   // Simulation
//   private static final double MAX_ARM_ORIENTATION_OFFSET = Math.toRadians(180.0);
//   private static final double MAX_CHEST_ORIENTATION_OFFSET = Math.toRadians(180.0);
//   private static final double MAX_PELVIS_ORIENTATION_OFFSET = Math.toRadians(180.0);

   // Hardware
   private static final double MAX_ARM_ORIENTATION_OFFSET = Math.toRadians(70.0);
   private static final double MAX_CHEST_ORIENTATION_OFFSET = Math.toRadians(35.0);
   private static final double MAX_PELVIS_ORIENTATION_OFFSET = Math.toRadians(35.0);

   public static boolean OVERRIDE_MESSAGE = false;
   public static final boolean ENABLE_POSTURE_OBJECTIVE = false;
   public static final boolean ENABLE_CONTACT_OBJECTIVE = false;
   public static final boolean INCLUDE_FF_VELOCITY = false;
   public static final Vector3D OVERRIDE_NORMAL = new Vector3D(Axis3D.Z);

   private static final double KP_ORIENTATION = 1200.0;
   private static final double MAX_CONTACT_POINT_ADJUSTMENT = 0.15;
   private static final double MAX_ORIENTATION_ERROR = Math.toRadians(5.0);
//   private static final double MAX_COM_Z_ERROR = 0.01;

   private final WholeBodyContactState wholeBodyContactState;
   private final StabilityMarginRegionCalculator multiContactRegionCalculator;
   private final SensitivityBasedStabilityGradientCalculator stabilityGradientCalculator;
   private final FullHumanoidRobotModel fullRobotModel;
   private final DoubleProvider minStabilityMargin;

   private final Vector3D chestDefaultWeight = new Vector3D();
   private final Vector3D chestMultiContactWeight = new Vector3D();
   private final YoVector3D chestWeight;
   private final SelectionMatrix3D chestSelectionMatrix = new SelectionMatrix3D();

   private final Vector3D pelvisDefaultWeight = new Vector3D();
   private final Vector3D pelvisMultiContactWeight = new Vector3D();
   private final YoVector3D pelvisWeight;
   private final SelectionMatrix3D pelvisSelectionMatrix = new SelectionMatrix3D();

   private final DefaultPID3DGains orientationGains = new DefaultPID3DGains();

   /* When enabled, will compute and update region under two conditions: contact adjustment is requested or upper body is load-bearing */
   private final YoBoolean isEnabled = new YoBoolean("isStabilityObjectiveEnabled", registry);
   /* The upper body is actively load bearing */
   private final BooleanProvider isUpperBodyLoadBearing;
   /* Compute support region and preview it at the current configuration, even if upper body is not load-bearing */
   private final YoBoolean requestContactAdjustment = new YoBoolean("requestContactAdjustment", registry);
   /* Optimize posture */
   private final YoBoolean requestPostureAdjustment = new YoBoolean("requestPostureAdjustment", registry);
   /* Contact normal for which support region is previewed */
   private final FrameVector3D regionNormal = new FrameVector3D();

   /* Reference frame of the region */
   private final PoseReferenceFrame regionFrame = new PoseReferenceFrame("regionFrame", ReferenceFrame.getWorldFrame());
   /* Region polygon, expressed in local frame */
   private final ConvexPolygon2D regionPolygon = new ConvexPolygon2D();
   /* Polygon scaler, to add a safety factor to the perceived region */
   private final ConvexPolygonScaler polygonScaler = new ConvexPolygonScaler();

   private final YoEnum<PostureOptimizerState> postureOptimizerState = new YoEnum<>("postureOptimizerState", registry, PostureOptimizerState.class);

   private final OrientationRetargeting chestOrientationRetargeting;
   private final OrientationRetargeting pelvisOrientationRetargeting;
   private final OrientationRetargeting bracingHandOrientationRetargeting;
   private final CoMHeightRetargeting comHeightRetargeting;

   private final JointBasics[] controlledJoints;
   private final DMatrixRMaj currentWholeBodyVelocity = new DMatrixRMaj(0);

   private final YoDouble stabilityMarginThreshold = new YoDouble("stabilityMarginThreshold", registry);
   private final YoDouble stabilityMarginHysteresis = new YoDouble("stabilityMarginHysteresis", registry);
   private final YoDouble sensitivityThresholdUpper = new YoDouble("sensitivityThresholdUpper", registry);
   private final YoDouble sensitivityThresholdLower = new YoDouble("sensitivityThresholdLower", registry);
   private final YoDouble alphaEnabled = new YoDouble("alphaEnabled", registry);

   private final YoBoolean addContactAdjustment = new YoBoolean("addContactAdjustment", registry);

   private final YoDouble kpPosture = new YoDouble("kpPosture", registry);
   private final YoDouble kpContact = new YoDouble("kpContact", registry);

   private final double updateDT;
   private final YoDouble previousStabilityMargin = new YoDouble("previousStabilityMargin", registry);
   private final YoDouble actualStabilityMarginVelocity = new YoDouble("actualStabilityMarginVelocity", registry);

   private final DMatrixRMaj scaledStabilityGradient = new DMatrixRMaj(0);

   private int bracingPointIndex;
   private final YoFrameVector3D contactPointAdjustment;
   private final YoFrameVector3D integratedContactPointAdjustment;

   private final FramePose3D tempPose = new FramePose3D();
   private final FramePoint3D tempPoint = new FramePoint3D();
   private final Point2D tempPoint2D = new Point2D();

   public StabilityBasedKinematicRetargetingCalculator(WholeBodyContactState wholeBodyContactState,
                                                       StabilityMarginRegionCalculator multiContactRegionCalculator,
                                                       FullHumanoidRobotModel fullRobotModel,
                                                       CentroidalMomentumCalculator centroidalMomentumCalculator,
                                                       ReferenceFrame midFeetZUpFrame,
                                                       ReferenceFrame centerOfMassFrame,
                                                       BooleanProvider isUpperBodyLoadBearing,
                                                       DoubleProvider minStabilityMargin,
                                                       double updateDT,
                                                       YoGraphicsListRegistry graphicsListRegistry,
                                                       YoRegistry parentRegistry)
   {
      this.wholeBodyContactState = wholeBodyContactState;
      this.multiContactRegionCalculator = multiContactRegionCalculator;
      this.fullRobotModel = fullRobotModel;
      this.controlledJoints = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel);
      this.updateDT = updateDT;
      int dofs = MultiBodySystemTools.computeDegreesOfFreedom(controlledJoints);
      currentWholeBodyVelocity.reshape(dofs, 1);

      this.isUpperBodyLoadBearing = isUpperBodyLoadBearing;
      this.minStabilityMargin = minStabilityMargin;

      stabilityMarginThreshold.set(0.15);
      stabilityMarginHysteresis.set(0.015);
      sensitivityThresholdLower.set(3.0e-4);
      sensitivityThresholdUpper.set(0.035);

      this.stabilityGradientCalculator = new SensitivityBasedStabilityGradientCalculator(fullRobotModel,
                                                                                         wholeBodyContactState,
                                                                                         multiContactRegionCalculator,
                                                                                         centroidalMomentumCalculator,
                                                                                         graphicsListRegistry,
                                                                                         registry);

      kpPosture.set(12.0); // 15.0);
      kpContact.set(0.25);

      double maxRate = Math.toRadians(15.0);
      chestOrientationRetargeting = new OrientationRetargeting("chest",  fullRobotModel.getChest(), updateDT, maxRate, MAX_CHEST_ORIENTATION_OFFSET, graphicsListRegistry, registry);
      pelvisOrientationRetargeting = new OrientationRetargeting("pelvis", fullRobotModel.getPelvis(), updateDT, maxRate, MAX_PELVIS_ORIENTATION_OFFSET, graphicsListRegistry, registry);
      bracingHandOrientationRetargeting = new OrientationRetargeting("bracingHand", fullRobotModel.getHand(HumanoidKinematicsToolboxController.BRACING_HAND_SIDE), updateDT, maxRate, MAX_ARM_ORIENTATION_OFFSET, graphicsListRegistry, registry);

      double minOffset = -0.07;
      double maxOffset = 0.04;
      comHeightRetargeting = new CoMHeightRetargeting(updateDT, 0.1, minOffset, maxOffset, fullRobotModel.getTotalMass(), centerOfMassFrame, stabilityGradientCalculator.getCentroidalMomentumCalculator(), registry);

      contactPointAdjustment = new YoFrameVector3D("contactPointAdjustment", ReferenceFrame.getWorldFrame(), registry);
      integratedContactPointAdjustment = new YoFrameVector3D("integratedContactPointAdjustment", ReferenceFrame.getWorldFrame(), registry);

      if (OVERRIDE_MESSAGE)
         isEnabled.set(ENABLE_POSTURE_OBJECTIVE || ENABLE_CONTACT_OBJECTIVE);

      if (OVERRIDE_MESSAGE)
         requestContactAdjustment.set(ENABLE_CONTACT_OBJECTIVE);

      if (OVERRIDE_MESSAGE)
         requestPostureAdjustment.set(ENABLE_POSTURE_OBJECTIVE);
      else
         requestPostureAdjustment.set(true);

      chestWeight = new YoVector3D("chestWeight", registry);
      pelvisWeight = new YoVector3D("pelvisWeight", registry);

      // hard-coded for overriding
      if (OVERRIDE_MESSAGE)
      {
         regionNormal.set(OVERRIDE_NORMAL);
      }

      // Detune the default KST values a bit
      chestDefaultWeight.set(0.0, 0.0, 0.5);
      pelvisDefaultWeight.set(1.0, 1.0, 1.0);

      // Custom lower weights to increase sensitivity
      chestMultiContactWeight.set(0.0, 0.0, 0.1);
      pelvisMultiContactWeight.set(0.2, 0.2, 0.2);

      orientationGains.setProportionalGains(KP_ORIENTATION);
      orientationGains.setMaxProportionalError(MAX_ORIENTATION_ERROR);

//      boolean useOldOrientation = false;
//      if (useOldOrientation)
//      {
//         orientationGains.setMaxProportionalError(100.0);
//         chestMultiContactWeight.set(chestDefaultWeight);
//         pelvisMultiContactWeight.set(pelvisDefaultWeight);
//      }
      
      addContactAdjustment.set(true);

      parentRegistry.addChild(registry);
   }

   public void setEnabled(boolean enable)
   {
      if (!OVERRIDE_MESSAGE)
         isEnabled.set(enable);
   }

   public boolean isEnabled()
   {
      return isEnabled.getBooleanValue();
   }

   public double getAlphaEnabled()
   {
      return alphaEnabled.getValue();
   }

   public double getPostureSensitivity()
   {
      return stabilityGradientCalculator.getPostureSensitivity();
   }

   public FrameVector3DReadOnly getContactAdjustmentNormal()
   {
      return regionNormal;
   }

   public void initialize()
   {
      chestOrientationRetargeting.initialize();
      pelvisOrientationRetargeting.initialize();
      bracingHandOrientationRetargeting.initialize();
      comHeightRetargeting.initialize();

      // update chest orientation
      chestWeight.set(isUpperBodyLoadBearing.getValue() ? chestMultiContactWeight : chestDefaultWeight);
      configureSelectionMatrix(chestSelectionMatrix, chestWeight);

      // update pelvis orientation
      pelvisWeight.set(isUpperBodyLoadBearing.getValue() ? pelvisMultiContactWeight : pelvisDefaultWeight);
      configureSelectionMatrix(pelvisSelectionMatrix, pelvisWeight);

      integratedContactPointAdjustment.setToZero();
      alphaEnabled.set(0.0);

      previousStabilityMargin.setToNaN();
      postureOptimizerState.set(PostureOptimizerState.NOMINAL);

      stabilityGradientCalculator.initialize();
   }

   public void update()
   {
      if (multiContactRegionCalculator.hasSolvedWholeRegion())
      {
         if (requestContactAdjustment.getValue())
         {
            RigidBodyBasics bracingHand = fullRobotModel.getHand(HumanoidKinematicsToolboxController.BRACING_HAND_SIDE);
            bracingPointIndex = wholeBodyContactState.indexOf(bracingHand);

            if (bracingPointIndex != -1)
            {
               contactPointAdjustment.set(stabilityGradientCalculator.computeContactPointAdjustment(HumanoidKinematicsToolboxController.BRACING_HAND_SIDE));

               if (!isUpperBodyLoadBearing.getValue() && contactPointAdjustment.normSquared() > 1.0e-5)
               {
                  contactPointAdjustment.scale(kpContact.getValue());
                  integratedContactPointAdjustment.add(updateDT * contactPointAdjustment.getX(), updateDT * contactPointAdjustment.getY(), updateDT * contactPointAdjustment.getZ());
               }
            }
         }
         else
         {
            integratedContactPointAdjustment.setToZero();
         }

         // always compute, for debugging
         stabilityGradientCalculator.computePostureAdjustment();

         // Save current whole-body velocity
         MultiBodySystemTools.extractJointsState(controlledJoints, JointStateType.VELOCITY, currentWholeBodyVelocity);

         // update gradient confidence
         if (!previousStabilityMargin.isNaN())
         {
            double deltaMargin = multiContactRegionCalculator.getStabilityMargin() - previousStabilityMargin.getValue();
            actualStabilityMarginVelocity.set(EuclidCoreTools.clamp(deltaMargin / updateDT, 0.6));
         }

         if (!isEnabled.getValue() || !isUpperBodyLoadBearing.getValue() || !requestPostureAdjustment.getValue() || multiContactRegionCalculator.getStabilityMargin() > getUpperMarginThreshold() || getPostureSensitivity() < sensitivityThresholdLower.getValue())
         {
            postureOptimizerState.set(PostureOptimizerState.NOMINAL);
         }
         else if (multiContactRegionCalculator.getStabilityMargin() > getLowerMarginThreshold() || stabilityGradientCalculator.getPostureSensitivity() < sensitivityThresholdUpper.getValue())
         {
            postureOptimizerState.set(PostureOptimizerState.FREEZE);
         }
         else
         {
            postureOptimizerState.set(PostureOptimizerState.OPTIMIZER);

            scaledStabilityGradient.set(stabilityGradientCalculator.getStabilityBoundaryGradient());
            CommonOps_DDRM.scale(kpPosture.getValue(), scaledStabilityGradient);
            MultiBodySystemTools.insertJointsState(controlledJoints, JointStateType.VELOCITY, scaledStabilityGradient);
            fullRobotModel.updateFrames();
         }
      }

      chestOrientationRetargeting.update(postureOptimizerState.getValue());
      pelvisOrientationRetargeting.update(postureOptimizerState.getValue());
      bracingHandOrientationRetargeting.update(postureOptimizerState.getValue());
      comHeightRetargeting.update(postureOptimizerState.getValue(), scaledStabilityGradient);

      if (multiContactRegionCalculator.hasSolvedWholeRegion())
      {
         // Reset to initial velocities
         MultiBodySystemTools.insertJointsState(controlledJoints, JointStateType.VELOCITY, currentWholeBodyVelocity);
         previousStabilityMargin.set(multiContactRegionCalculator.getStabilityMargin());

         if (postureOptimizerState.getValue() == PostureOptimizerState.OPTIMIZER)
            fullRobotModel.updateFrames();
      }
   }

   private double getUpperMarginThreshold()
   {
      return stabilityMarginThreshold.getValue() + stabilityMarginHysteresis.getValue();
   }

   private double getLowerMarginThreshold()
   {
      return stabilityMarginThreshold.getValue() - stabilityMarginHysteresis.getValue();
   }

   private static void configureSelectionMatrix(SelectionMatrix3D selectionMatrix, Tuple3DReadOnly weight)
   {
      double weightThreshold = 1.0e-3;
      selectionMatrix.selectXAxis(weight.getX() > weightThreshold);
      selectionMatrix.selectYAxis(weight.getY() > weightThreshold);
      selectionMatrix.selectZAxis(weight.getZ() > weightThreshold);
   }

   public FrameVector3DReadOnly getIntegratedContactPointAdjustment()
   {
      return integratedContactPointAdjustment;
   }

   public void disableContactAdjustment()
   {
      if (!OVERRIDE_MESSAGE)
         requestContactAdjustment.set(false);
   }

   public void enableContactAdjustment(Tuple3DReadOnly regionPoint, Orientation3DReadOnly regionOrientation, ConvexPolygon2DReadOnly regionPolygon, Tuple3DReadOnly regionNormal)
   {
//      LogTools.info("Enabling contact adjustment");

      if (!OVERRIDE_MESSAGE)
      {
         requestContactAdjustment.set(true);
         tempPose.set(regionPoint, regionOrientation);
         regionFrame.setPoseAndUpdate(tempPose);

         this.regionPolygon.set(regionPolygon);
         this.regionNormal.set(regionNormal);

         if (!regionPolygon.isEmpty())
         {
            double safetyDistance = 0.07;
            polygonScaler.scaleConvexPolygon(regionPolygon, safetyDistance, this.regionPolygon);
         }
      }
   }

   public boolean contactAdjustmentRequested()
   {
      return requestContactAdjustment.getValue();
   }

   public void retargetRigidBody(KinematicsToolboxRigidBodyCommand command)
   {
      boolean isBracingHand = command.getEndEffector().equals(fullRobotModel.getHand(HumanoidKinematicsToolboxController.BRACING_HAND_SIDE));
      if (isBracingHand)
      {
         // Posture adjustment
         FixedFrameQuaternionBasics desiredHandOrientation = command.getDesiredPose().getOrientation();
         bracingHandOrientationRetargeting.updateNominalOrientation(desiredHandOrientation, command.getControlFramePose());

         desiredHandOrientation.set(bracingHandOrientationRetargeting.getDesiredOrientation());
         command.setHasDesiredVelocity(true);
         command.getDesiredVelocity().getLinearPart().setToZero();
         if (INCLUDE_FF_VELOCITY)
            command.getDesiredVelocity().getAngularPart().add(bracingHandOrientationRetargeting.getAngularVelocity());

         // Contact point adjustment
         if (!isUpperBodyLoadBearing.getValue())
         {
            // Position of the user
            tempPoint.setIncludingFrame(command.getDesiredPose().getPosition());

            // Retargeted position
            tempPoint.add(integratedContactPointAdjustment);

            // Cap adjustment
            double adjustmentSquared = integratedContactPointAdjustment.normSquared();
            double maxAdjustmentSquared = EuclidCoreTools.square(MAX_CONTACT_POINT_ADJUSTMENT);
            if (adjustmentSquared > maxAdjustmentSquared)
            {
               integratedContactPointAdjustment.scale(maxAdjustmentSquared / adjustmentSquared);
            }

            if (!regionPolygon.isEmpty())
            { // Snap to region
               tempPoint.changeFrame(regionFrame);
               tempPoint2D.set(tempPoint);
               regionPolygon.orthogonalProjection(tempPoint2D);
               tempPoint.set(tempPoint2D);
               tempPoint.changeFrame(ReferenceFrame.getWorldFrame());

               // Subtract off snapped difference
               integratedContactPointAdjustment.sub(tempPoint, command.getDesiredPose().getPosition());
            }

            // Add it
            if (addContactAdjustment.getValue())
               command.getDesiredPose().getPosition().add(integratedContactPointAdjustment);
         }
      }
   }

   public void addOrientationObjectives(FeedbackControlCommandBuffer bufferToPack)
   {
      if (!KSTStreamingState.USE_DEFAULT_ORIENTATION_OBJECTIVES)
      {
         OrientationFeedbackControlCommand pelvisOrientationCommand = bufferToPack.addOrientationFeedbackControlCommand();
         pelvisOrientationCommand.set(fullRobotModel.getElevator(), fullRobotModel.getPelvis());
         pelvisOrientationCommand.setWeightsForSolver(pelvisWeight);
         pelvisOrientationCommand.setSelectionMatrix(pelvisSelectionMatrix);
         pelvisOrientationCommand.setGains(orientationGains);
         pelvisOrientationCommand.setInverseKinematics(pelvisOrientationRetargeting.getDesiredOrientation(), null);
         if (INCLUDE_FF_VELOCITY)
            pelvisOrientationCommand.getReferenceAngularVelocity().set(pelvisOrientationRetargeting.getAngularVelocity());

         OrientationFeedbackControlCommand chestOrientationCommand = bufferToPack.addOrientationFeedbackControlCommand();
         chestOrientationCommand.set(fullRobotModel.getRootBody(), fullRobotModel.getChest());
         chestOrientationCommand.setWeightsForSolver(chestWeight);
         chestOrientationCommand.setSelectionMatrix(chestSelectionMatrix);
         chestOrientationCommand.setGains(orientationGains);
         chestOrientationCommand.setInverseKinematics(chestOrientationRetargeting.getDesiredOrientation(), null);
         if (INCLUDE_FF_VELOCITY)
            chestOrientationCommand.getReferenceAngularVelocity().set(chestOrientationRetargeting.getAngularVelocity());
      }
   }

   public void retargetCenterOfMass(KinematicsToolboxCenterOfMassCommand command)
   {
      comHeightRetargeting.updateNominalHeight(command.getDesiredPosition().getZ());
      command.getDesiredPosition().setZ(comHeightRetargeting.getHeight());
   }

   private double updateAlphaEnabled()
   {
      double stabilityMargin = multiContactRegionCalculator.getStabilityMargin();
      double deltaStabilityMargin = stabilityMargin - minStabilityMargin.getValue();
      double enabledFraction = EuclidCoreTools.clamp(1.0 - deltaStabilityMargin / (stabilityMarginThreshold.getValue() - minStabilityMargin.getValue()), 0.0, 1.0);
      alphaEnabled.set(enabledFraction);
      return alphaEnabled.getValue();
   }
}
