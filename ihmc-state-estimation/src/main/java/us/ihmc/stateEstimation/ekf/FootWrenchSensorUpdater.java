package us.ihmc.stateEstimation.ekf;

import us.ihmc.ekf.filter.FilterTools;
import us.ihmc.ekf.filter.sensor.Sensor;
import us.ihmc.ekf.filter.sensor.implementations.FootVelocitySensor;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFramePoint;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

/**
 * Helper class that converts a measured foot wrench into a "zero-linear-velocity" sensor for the CoP location.
 *
 * @author Georg Wiedebach
 */
public class FootWrenchSensorUpdater
{
   private static final String parameterGroup = "Foot";

   private final double weight;
   private final AlphaFilteredYoFrameVector filteredForce;
   private final ReferenceFrame copFrame;

   private final FootVelocitySensor footVelocitySensor;

   private final DoubleProvider weightThresholdForTrust;

   public FootWrenchSensorUpdater(RigidBodyBasics foot, ReferenceFrame soleFrame, double dt, double weight, YoGraphicsListRegistry graphicsListRegistry,
                                  YoVariableRegistry registry)
   {
      this.weight = weight;

      weightThresholdForTrust = FilterTools.findOrCreate(parameterGroup + "WeightThresholdForTrust", registry, 0.3);
      DoubleProvider forceFilter = FilterTools.findOrCreate(parameterGroup + "WrenchFilter", registry, 100.0);
      DoubleProvider forceAlpha = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(forceFilter.getValue(), dt);

      filteredForce = new AlphaFilteredYoFrameVector(foot.getName() + "Force", "", registry, forceAlpha, ReferenceFrame.getWorldFrame());
      AlphaFilteredYoFramePoint filteredCoP = new AlphaFilteredYoFramePoint(soleFrame.getName() + "CoPPositionInSole", "", registry, forceAlpha, soleFrame);
      YoFramePoint3D yoCopPosition = new YoFramePoint3D(soleFrame.getName() + "CoPPosition", ReferenceFrame.getWorldFrame(), registry);

      copFrame = new ReferenceFrame(soleFrame.getName() + "CopFrame", soleFrame)
      {
         private final FramePoint3D copPosition = new FramePoint3D();

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            wrench.changeFrame(getParent());

            // If the measured weight is lower then 1% of the robot weight just set the cop to zero.
            if (wrench.getLinearPartZ() > weight * 0.01)
            {
               double copX = -wrench.getAngularPartY() / wrench.getLinearPartZ();
               double copY = wrench.getAngularPartX() / wrench.getLinearPartZ();
               copPosition.setIncludingFrame(getParent(), copX, copY, 0.0);
               filteredCoP.update(copPosition);
               yoCopPosition.setMatchingFrame(filteredCoP);
            }
            else
            {
               copPosition.setToZero(getParent());
               filteredCoP.set(copPosition);
               yoCopPosition.setToNaN();
            }

            transformToParent.setTranslationAndIdentityRotation(filteredCoP);
         }
      };

      if (graphicsListRegistry != null)
      {
         YoGraphicPosition copViz = new YoGraphicPosition(copFrame.getName(), yoCopPosition, 0.01, YoAppearance.Green());
         graphicsListRegistry.registerYoGraphic("EKF", copViz);
         graphicsListRegistry.registerArtifact("EKF", copViz.createArtifact());
      }

      footVelocitySensor = new FootVelocitySensor(dt, foot, copFrame, parameterGroup, registry);
   }

   private int loadedCount = 0;
   private final Wrench wrench = new Wrench();

   public void update(WrenchReadOnly footWrench, boolean fixRobot)
   {
      wrench.setIncludingFrame(footWrench);
      copFrame.update();

      wrench.changeFrame(ReferenceFrame.getWorldFrame());
      filteredForce.update(wrench.getLinearPart());

      if (fixRobot)
      {
         footVelocitySensor.setLoad(1.0);
      }
      else if (loadedCount > 20)
      {
         footVelocitySensor.setLoad(1.0);
      }
      else
      {
         footVelocitySensor.setLoad(0.0);
      }

      boolean loaded = filteredForce.getZ() / weight > weightThresholdForTrust.getValue();
      if (!loaded)
      {
         loadedCount = 0;
      }
      else
      {
         loadedCount++;
      }
   }

   public Sensor getFootLinearVelocitySensor()
   {
      return footVelocitySensor;
   }
}
