package us.ihmc.commonWalkingControlModules.capturePoint;

import static us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools.*;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Black;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.BlueViolet;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Yellow;

import us.ihmc.commonWalkingControlModules.messageHandlers.CenterOfMassTrajectoryHandler;
import us.ihmc.commonWalkingControlModules.messageHandlers.MomentumTrajectoryHandler;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchDistributorTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DBasics;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.robotics.math.filters.AlphaFilteredTuple2D;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class PrecomputedICPPlanner
{
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

   private final YoFramePoint3D yoDesiredCoPPosition = new YoFramePoint3D(name + "DesiredCoPPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D yoDesiredCMPPosition = new YoFramePoint3D(name + "DesiredCMPPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D yoDesiredCoMPosition = new YoFramePoint3D(name + "DesiredCoMPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D yoDesiredICPPosition = new YoFramePoint3D(name + "DesiredICPPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D yoDesiredICPVelocity = new YoFrameVector3D(name + "DesiredICPVelocity", ReferenceFrame.getWorldFrame(), registry);

   private final YoBoolean currentlyBlendingICPTrajectories = new YoBoolean("currentlyBlendingICPTrajectories", registry);
   private final YoBoolean isBlending = new YoBoolean("isBlending", registry);
   private final YoDouble blendingStartTime = new YoDouble("blendingStartTime", registry);
   private final YoDouble blendingDuration = new YoDouble("blendingDuration", registry);

   private final YoDouble omega0 = new YoDouble(name + "Omega0", registry);

   private final FramePoint3D desiredCoPPosition = new FramePoint3D();
   private final FramePoint3D desiredCMPPosition = new FramePoint3D();
   private final FramePoint3D desiredCoMPosition = new FramePoint3D();
   private final FrameVector3D desiredCoMVelocity = new FrameVector3D();
   private final FrameVector3D desiredCoMAcceleration = new FrameVector3D();
   private final FramePoint3D desiredICPPosition = new FramePoint3D();
   private final FrameVector3D desiredICPVelocity = new FrameVector3D();
   private final FrameVector3D filteredDesiredICPVelocity = new FrameVector3D();

   private final FrameVector3D desiredAngularMomentum = new FrameVector3D();
   private final FrameVector3D desiredAngularMomentumRate = new FrameVector3D();

   private final CenterOfMassTrajectoryHandler centerOfMassTrajectoryHandler;
   private final MomentumTrajectoryHandler momentumTrajectoryHandler;

   private double mass;
   private double gravity;

   private final DoubleParameter filterBreakFrequency = new DoubleParameter("PrecomputedICPVelocityFilterBreakFrequency", registry, 5.0);
   private final DoubleProvider alphaProvider;
   private final Tuple2DBasics filteredPrecomputedIcpVelocity;

   private final FramePoint2D tempICPPosition = new FramePoint2D();
   private final FramePoint2D tempCoPPosition = new FramePoint2D();

   public PrecomputedICPPlanner(CenterOfMassTrajectoryHandler centerOfMassTrajectoryHandler, MomentumTrajectoryHandler momentumTrajectoryHandler,
                                YoRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(Double.NaN, centerOfMassTrajectoryHandler, momentumTrajectoryHandler, parentRegistry, yoGraphicsListRegistry);
   }

   public PrecomputedICPPlanner(double dt, CenterOfMassTrajectoryHandler centerOfMassTrajectoryHandler, MomentumTrajectoryHandler momentumTrajectoryHandler,
                                YoRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.centerOfMassTrajectoryHandler = centerOfMassTrajectoryHandler;
      this.momentumTrajectoryHandler = momentumTrajectoryHandler;
      blendingDuration.set(0.5);

      if (!Double.isNaN(dt))
      {
         alphaProvider = new DoubleProvider()
         {
            @Override
            public double getValue()
            {
               return AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(filterBreakFrequency.getValue(), dt);
            }
         };
         filteredPrecomputedIcpVelocity = new AlphaFilteredTuple2D(alphaProvider);
      }
      else
      {
         alphaProvider = null;
         filteredPrecomputedIcpVelocity = new Vector2D();
      }

      parentRegistry.addChild(registry);

      if (yoGraphicsListRegistry != null)
      {
         YoGraphicsList yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());
         ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());
         
         YoGraphicPosition desiredICPPositionGraphic = new YoGraphicPosition("Desired ICP Precomputed", yoDesiredICPPosition, 0.005, Yellow(),
                                                                             GraphicType.BALL_WITH_ROTATED_CROSS);
         yoGraphicsList.add(desiredICPPositionGraphic);
         artifactList.add(desiredICPPositionGraphic.createArtifact());
         
         YoGraphicPosition desiredCenterOfMassPositionViz = new YoGraphicPosition("Desired CoM Precomputed", yoDesiredCoMPosition, 0.003, Black(),
                                                                                  GraphicType.BALL_WITH_ROTATED_CROSS);
         yoGraphicsList.add(desiredCenterOfMassPositionViz);
         artifactList.add(desiredCenterOfMassPositionViz.createArtifact());
         
         YoGraphicPosition desiredCoPPositionViz = new YoGraphicPosition("Perfect CoP Precomputed", yoDesiredCoPPosition, 0.005, BlueViolet(),
                                                                         GraphicType.DIAMOND);
         YoGraphicPosition desiredCMPPositionViz = new YoGraphicPosition("Perfect CMP Precomputed", yoDesiredCMPPosition, 0.005, BlueViolet());
         
         artifactList.add(desiredCoPPositionViz.createArtifact());
         yoGraphicsList.add(desiredCMPPositionViz);
         artifactList.add(desiredCMPPositionViz.createArtifact());
         
         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
         yoGraphicsListRegistry.registerArtifactList(artifactList);
      }

      hideViz();
   }

   private void compute(double time)
   {
      double omega0 = this.omega0.getDoubleValue();
      centerOfMassTrajectoryHandler.packDesiredICPAtTime(time, omega0, desiredICPPosition, desiredICPVelocity, desiredCoMPosition);

      filteredPrecomputedIcpVelocity.set(desiredICPVelocity);
      filteredDesiredICPVelocity.set(filteredPrecomputedIcpVelocity);

      computeCentroidalMomentumPivot(desiredICPPosition, filteredDesiredICPVelocity, omega0, desiredCMPPosition);
      computeCenterOfMassVelocity(desiredCoMPosition, desiredICPPosition, omega0, desiredCoMVelocity);
      computeCenterOfMassAcceleration(desiredCoMVelocity, filteredDesiredICPVelocity, omega0, desiredCoMAcceleration);
      double comZAcceleration = desiredCoMAcceleration.getZ();

      desiredCoPPosition.set(desiredCMPPosition);
      // Can compute CoP if we have a momentum rate of change otherwise set it to match the CMP.
      if (momentumTrajectoryHandler != null && momentumTrajectoryHandler.packDesiredAngularMomentumAtTime(time, desiredAngularMomentum, desiredAngularMomentumRate))
      {
         double fZ = WrenchDistributorTools.computeFz(mass, gravity, comZAcceleration);
         desiredCoPPosition.addX(-desiredAngularMomentumRate.getY() / fZ);
         desiredCoPPosition.addY(desiredAngularMomentumRate.getX() / fZ);
      }

      yoDesiredICPPosition.set(desiredICPPosition);
      yoDesiredICPVelocity.set(desiredICPVelocity);
      yoDesiredCoMPosition.set(desiredCoMPosition);
      yoDesiredCoPPosition.set(desiredCoPPosition);
      yoDesiredCMPPosition.set(desiredCMPPosition);
   }

   public void compute(double time,
                       FramePoint2DBasics desiredCapturePoint2dToPack,
                       FrameVector2DBasics desiredCapturePointVelocity2dToPack,
                       FramePoint2DBasics desiredCoP2DToPack)
   {
      if (isWithinInterval(time))
      {
         compute(time);
         desiredCapturePoint2dToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), desiredICPPosition);
         desiredCapturePointVelocity2dToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), filteredPrecomputedIcpVelocity);
         desiredCoP2DToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), desiredCoPPosition);
      }
      else
      {
         hideViz();
      }
      currentlyBlendingICPTrajectories.set(false);

      centerOfMassTrajectoryHandler.clearPointsInPast();
      momentumTrajectoryHandler.clearPointsInPast();
   }

   public void computeAndBlend(double time, FixedFramePoint2DBasics desiredCapturePoint2dToPack, FixedFrameVector2DBasics desiredCapturePointVelocity2dToPack,
                               FixedFramePoint2DBasics desiredCenterOfPressure2dToPack)
   {
      desiredCapturePoint2dToPack.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      desiredCapturePointVelocity2dToPack.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      desiredCenterOfPressure2dToPack.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      if (isWithinInterval(time))
      {
         compute(time);
         if (!currentlyBlendingICPTrajectories.getBooleanValue())
         {
            blendingStartTime.set(time);
            currentlyBlendingICPTrajectories.set(true);
         }

         double alpha = (time - blendingStartTime.getDoubleValue()) / blendingDuration.getDoubleValue();
         isBlending.set(alpha < 1.0);
         alpha = MathTools.clamp(alpha, 0.0, 1.0);

         tempICPPosition.set(desiredICPPosition);
         tempCoPPosition.set(desiredCoPPosition);

         desiredCapturePoint2dToPack.interpolate(desiredCapturePoint2dToPack, tempICPPosition, alpha);
         desiredCapturePointVelocity2dToPack.interpolate(desiredCapturePointVelocity2dToPack, filteredPrecomputedIcpVelocity, alpha);
         desiredCenterOfPressure2dToPack.interpolate(desiredCenterOfPressure2dToPack, tempCoPPosition, alpha);
      }
      else
      {
         isBlending.set(false);
         currentlyBlendingICPTrajectories.set(false);
         hideViz();
      }

      centerOfMassTrajectoryHandler.clearPointsInPast();
      momentumTrajectoryHandler.clearPointsInPast();
   }

   public boolean isWithinInterval(double time)
   {
      return centerOfMassTrajectoryHandler.isWithinInterval(time);
   }

   /**
    * Intrinsic robot parameter.
    * <p>
    * Correspond the natural frequency response of the robot when modeled as an inverted pendulum:
    * {@code omega0 = Math.sqrt(g / z0)}, where {@code g} is equal to the magnitude of the gravity,
    * and {@code z0} is the constant center of mass height of the robot with respect to is feet.
    * </p>
    *
    * @param omega0 the robot's natural frequency.
    */
   public void setOmega0(double omega0)
   {
      this.omega0.set(omega0);
   }

   public void setMass(double mass)
   {
      this.mass = mass;
   }

   public void setGravity(double gravity)
   {
      this.gravity = gravity;
   }

   private void hideViz()
   {
      yoDesiredICPPosition.setToNaN();
      yoDesiredICPVelocity.setToNaN();
      yoDesiredCoMPosition.setToNaN();
      yoDesiredCMPPosition.setToNaN();
   }
}
