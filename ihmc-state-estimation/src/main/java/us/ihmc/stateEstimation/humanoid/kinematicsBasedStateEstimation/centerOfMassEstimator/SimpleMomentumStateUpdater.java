package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.centerOfMassEstimator;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.DefaultPoint2DGraphic;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimpleMomentumStateUpdater implements MomentumStateUpdater
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final boolean VISUALIZE = false;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final CenterOfMassJacobian centerOfMassJacobianWorld;

   private final YoFramePoint3D yoCenterOfMassPosition;
   private final YoFrameVector3D yoCenterOfMassVelocityUsingPelvisAndKinematics;
   private final YoFrameVector3D yoCenterOfMassVelocityIntegrateGRF;
   private final YoFrameVector3D yoCenterOfMassVelocity;

   private final YoFrameVector3D totalGroundReactionForce = new YoFrameVector3D("totalGroundForce", worldFrame, registry);
   private final YoDouble robotMass = new YoDouble("robotMass", registry);
   private final YoFrameVector3D comAcceleration = new YoFrameVector3D("comAcceleration", worldFrame, registry);

   private final CenterOfMassDataHolder estimatorCenterOfMassDataHolderToUpdate;

   private final BooleanProvider useGroundReactionForcesToComputeCenterOfMassVelocity;
   private final DoubleProvider grfAgainstIMUAndKinematicsForVelocityBreakFrequency;

   private final List<RigidBodyBasics> feet = new ArrayList<>();
   private final Map<RigidBodyBasics, FootSwitchInterface> footSwitches;
   private final Map<RigidBodyBasics, Wrench> footWrenches = new LinkedHashMap<>();

   private final double estimatorDT;
   private final double gravitationalAcceleration;

   private final FrameVector3D comVelocityGRFPart = new FrameVector3D();
   private final FrameVector3D comVelocityPelvisAndKinPart = new FrameVector3D();
   private final FrameVector3D centerOfMassVelocityUsingPelvisIMUAndKinematics = new FrameVector3D(worldFrame);
   private final FrameVector3D tempCoMAcceleration = new FrameVector3D(worldFrame);
   private final FrameVector3D tempFootForce = new FrameVector3D(worldFrame);

   public SimpleMomentumStateUpdater(FloatingJointReadOnly rootJoint,
                                     double gravitationalAcceleration,
                                     StateEstimatorParameters stateEstimatorParameters,
                                     Map<RigidBodyBasics, FootSwitchInterface> footSwitches,
                                     CenterOfMassDataHolder estimatorCenterOfMassDataHolderToUpdate,
                                     YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.estimatorCenterOfMassDataHolderToUpdate = estimatorCenterOfMassDataHolderToUpdate;
      this.footSwitches = footSwitches;
      this.gravitationalAcceleration = gravitationalAcceleration;

      feet.addAll(footSwitches.keySet());
      feet.forEach(foot -> footWrenches.put(foot, new Wrench()));
      estimatorDT = stateEstimatorParameters.getEstimatorDT();

      MovingReferenceFrame rootJointFrame = rootJoint.getFrameAfterJoint();
      RigidBodyReadOnly elevator = rootJoint.getPredecessor();

      robotMass.set(TotalMassCalculator.computeSubTreeMass(elevator));

      useGroundReactionForcesToComputeCenterOfMassVelocity = new BooleanParameter("useGRFToComputeCoMVelocity",
                                                                                  registry,
                                                                                  stateEstimatorParameters.useGroundReactionForcesToComputeCenterOfMassVelocity());
      grfAgainstIMUAndKinematicsForVelocityBreakFrequency = new DoubleParameter("grfAgainstIMUAndKinematicsForVelocityBreakFrequency",
                                                                                registry,
                                                                                stateEstimatorParameters.getCenterOfMassVelocityFusingFrequency());

      centerOfMassJacobianWorld = new CenterOfMassJacobian(elevator, rootJointFrame);
      yoCenterOfMassPosition = new YoFramePoint3D("estimatedCenterOfMassPosition", worldFrame, registry);
      yoCenterOfMassVelocityUsingPelvisAndKinematics = new YoFrameVector3D("estimatedCenterOfMassVelocityPelvisAndKin", worldFrame, registry);
      yoCenterOfMassVelocityIntegrateGRF = new YoFrameVector3D("estimatedCenterOfMassVelocityGRF", worldFrame, registry);
      yoCenterOfMassVelocity = new YoFrameVector3D("estimatedCenterOfMassVelocity", worldFrame, registry);

      if (VISUALIZE)
      {
         if (yoGraphicsListRegistry != null)
         {
            YoArtifactPosition comArtifact = new YoGraphicPosition("Meas CoM",
                                                                   yoCenterOfMassPosition,
                                                                   0.006,
                                                                   YoAppearance.Black(),
                                                                   GraphicType.CROSS).createArtifact();
            yoGraphicsListRegistry.registerArtifact("StateEstimator", comArtifact);
         }
      }
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void update()
   {
      centerOfMassJacobianWorld.reset();
      yoCenterOfMassPosition.setMatchingFrame(centerOfMassJacobianWorld.getCenterOfMass());

      yoCenterOfMassVelocityUsingPelvisAndKinematics.setMatchingFrame(centerOfMassJacobianWorld.getCenterOfMassVelocity());

      if (useGroundReactionForcesToComputeCenterOfMassVelocity.getValue())
      {
         //centerOfMassVelocity at this point is from pelvis velocity and joint velocities, where pelvis velocity is estimated from pelvis imu at high freq and leg kinematics at low freq.

         //TODO: Lots of duplicated computation with reading the force sensors and changing frames. Just do it once and share...
         computeTotalGroundReactionForce();

         double totalMass = robotMass.getDoubleValue();
         if (totalMass < 0.01)
            totalMass = 0.01;

         comAcceleration.scale(1.0 / totalMass);

         comAcceleration.scale(estimatorDT);
         yoCenterOfMassVelocityIntegrateGRF.add(comAcceleration);

         comVelocityGRFPart.set(yoCenterOfMassVelocity);
         comVelocityGRFPart.add(comAcceleration);
         comAcceleration.scale(1.0 / estimatorDT);

         comVelocityPelvisAndKinPart.set(centerOfMassVelocityUsingPelvisIMUAndKinematics);

         double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(grfAgainstIMUAndKinematicsForVelocityBreakFrequency.getValue(),
                                                                                        estimatorDT);
         comVelocityGRFPart.scale(alpha);
         comVelocityPelvisAndKinPart.scale(1.0 - alpha);

         yoCenterOfMassVelocity.add(comVelocityGRFPart, comVelocityPelvisAndKinPart);
      }
      else
      {
         yoCenterOfMassVelocity.set(centerOfMassVelocityUsingPelvisIMUAndKinematics);
      }

      if (estimatorCenterOfMassDataHolderToUpdate != null)
         estimatorCenterOfMassDataHolderToUpdate.setCenterOfMassVelocity(yoCenterOfMassVelocity);
   }

   private void computeTotalGroundReactionForce()
   {
      totalGroundReactionForce.setToZero();

      for (int i = 0; i < feet.size(); i++)
      {
         RigidBodyBasics foot = feet.get(i);
         Wrench footWrench = footWrenches.get(foot);
         footSwitches.get(foot).getMeasuredWrench(footWrench);
         tempFootForce.setIncludingFrame(footWrench.getLinearPart());
         tempFootForce.changeFrame(worldFrame);

         totalGroundReactionForce.add(tempFootForce);
      }

      tempCoMAcceleration.set(totalGroundReactionForce);
      comAcceleration.set(tempCoMAcceleration);
      comAcceleration.setZ(comAcceleration.getZ() - robotMass.getDoubleValue() * gravitationalAcceleration);
   }

   public void getEstimatedCoMPosition(FramePoint3D comPositionToPack)
   {
      comPositionToPack.setIncludingFrame(yoCenterOfMassPosition);
   }

   public void getEstimatedCoMVelocity(FrameVector3D comVelocityToPack)
   {
      comVelocityToPack.setIncludingFrame(yoCenterOfMassVelocity);
   }

   @Override
   public YoRegistry getRegistry()
   {
      return registry;
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint2D("Meas CoM",
                                                                    yoCenterOfMassPosition,
                                                                    0.012,
                                                                    ColorDefinitions.Black(),
                                                                    DefaultPoint2DGraphic.CROSS));
      return group;
   }
}
