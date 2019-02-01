package us.ihmc.stateEstimation.ekf;

import us.ihmc.ekf.filter.FilterTools;
import us.ihmc.ekf.filter.sensor.Sensor;
import us.ihmc.ekf.filter.sensor.implementations.FootVelocitySensor;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

public class FootWrenchSensor
{
   private static final String parameterGroup = "Foot";

   private final double weight;
   private final AlphaFilteredYoFrameVector filteredForce;
   private final AlphaFilteredYoFrameVector filteredTorque;
   private final ReferenceFrame copFrame;

   private final FootVelocitySensor footVelocitySensor;

   public FootWrenchSensor(RigidBodyBasics foot, ReferenceFrame soleFrame, double dt, double weight, YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry registry)
   {
      this.weight = weight;

      DoubleProvider forceFilter = FilterTools.findOrCreate(parameterGroup + "ForceFilter", registry, 100.0);
      DoubleProvider forceAlpha = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(forceFilter .getValue(), dt);
      DoubleProvider torqueFilter = FilterTools.findOrCreate(parameterGroup + "TorqueFilter", registry, 100.0);
      DoubleProvider torqueAlpha = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(torqueFilter .getValue(), dt);

      filteredForce = new AlphaFilteredYoFrameVector(foot.getName() + "Force", "", registry, forceAlpha, ReferenceFrame.getWorldFrame());
      filteredTorque = new AlphaFilteredYoFrameVector(foot.getName() + "Torque", "", registry, torqueAlpha, ReferenceFrame.getWorldFrame());

      YoFramePoint3D yoCopPosition = new YoFramePoint3D(soleFrame.getName() + "CopFrame", ReferenceFrame.getWorldFrame(), registry);
      copFrame = new ReferenceFrame(soleFrame.getName() + "CopFrame", soleFrame)
      {
         private final FrameVector3D force = new FrameVector3D();
         private final FrameVector3D torque = new FrameVector3D();
         private final FramePoint3D copPosition = new FramePoint3D();

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            force.setIncludingFrame(filteredForce);
            torque.setIncludingFrame(filteredTorque);
            force.changeFrame(soleFrame);
            torque.changeFrame(soleFrame);

            // If the measured weight is lower then 1% of the robot weight just set the cop to zero.
            if (force.getZ() > weight * 0.01)
            {
               double copX = -torque.getY() / force.getZ();
               double copY = torque.getX() / force.getZ();
               copPosition.setIncludingFrame(getParent(), copX, copY, 0.0);
               yoCopPosition.setMatchingFrame(copPosition);
            }
            else
            {
               copPosition.setToZero(getParent());
               yoCopPosition.setToNaN();
            }

            transformToParent.setTranslationAndIdentityRotation(copPosition);
         }
      };

      if (graphicsListRegistry != null)
      {
         YoGraphicPosition copViz = new YoGraphicPosition(copFrame.getName(), yoCopPosition, 0.02, YoAppearance.Green());
         graphicsListRegistry.registerYoGraphic("EKF", copViz);
      }

      footVelocitySensor = new FootVelocitySensor(dt, foot, copFrame, parameterGroup, registry);
   }

   private final FrameVector3D tempVector = new FrameVector3D();
   private int loadedCount = 0;

   public void update(WrenchReadOnly footWrench, boolean fixRobot)
   {
      tempVector.setIncludingFrame(footWrench.getLinearPart());
      tempVector.changeFrame(ReferenceFrame.getWorldFrame());
      filteredForce.update(tempVector);

      tempVector.setIncludingFrame(footWrench.getAngularPart());
      tempVector.changeFrame(ReferenceFrame.getWorldFrame());
      filteredTorque.update(tempVector);

      copFrame.update();

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

      boolean loaded = filteredForce.getZ() / weight > 0.3;
      if (!loaded)
      {
         loadedCount = 0;
      }
      else
      {
         loadedCount++;
      }
   }

   public Sensor getSensor()
   {
      return footVelocitySensor;
   }
}
