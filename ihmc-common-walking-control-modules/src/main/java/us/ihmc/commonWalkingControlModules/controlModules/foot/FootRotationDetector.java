package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.awt.Color;

import us.ihmc.commonWalkingControlModules.momentumBasedController.ParameterProvider;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FrameLine3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine3DBasics;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFramePoint2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint2D;

/**
 * This class computes whether a foot is rotating.</br>
 * If that is the case provides an estimate of the line of rotation. This class does not rely on a center of pressure
 * estimate and can be used with robots that do not have force-torque sensing in the feet.
 * <p>
 * Two strategies can be used to detect the rotation:
 * <ul>
 * <li>Assuming flat ground and a ground height of 0.0 the foot can be intersected with this plane to accurately find
 * the line of rotation.<br>
 * <li>The twist of the foot can be used to detect an instantaneous line of rotation if the angular velocity of the foot
 * is sufficient.
 * </ul>
 * <p>
 * Both methods require some form of thresholding as to whether foot rotation is occurring. For the twist based
 * detection the angular foot velocity (only in sole plane) is integrated with a leak rate. This integrated velocity is
 * then thresholded to determine if foot the foot is unstable.
 * <p>
 * ------------</br>
 * Currently this class implements only the second one since the assumptions are less restrictive. Depending on the
 * performance the first method should be implemented as an alternative.</br>
 * ------------
 *
 * @author Georg Wiedebach
 */
public class FootRotationDetector
{
   private final YoVariableRegistry registry;

   private final YoFramePoint2D linePointA;
   private final YoFramePoint2D linePointB;

   private final double dt;
   private final MovingReferenceFrame soleFrame;

   private final AlphaFilteredYoFramePoint2d filteredPointOfRotation;
   private final AlphaFilteredYoFrameVector2d filteredAxisOfRotation;

   private final FrameLine2DBasics lineOfRotationInSole = new FrameLine2D();
   private final YoDouble integratedRotationAngle;
   private final YoBoolean isRotating;

   private final DoubleProvider omegaThresholdForEstimation;
   private final DoubleProvider decayBreakFrequency;
   private final DoubleProvider filterBreakFrequency;
   private final DoubleProvider rotationThreshold;

   public FootRotationDetector(RobotSide side, MovingReferenceFrame soleFrame, double dt, YoVariableRegistry parentRegistry,
                               YoGraphicsListRegistry graphicsRegistry)
   {
      this.soleFrame = soleFrame;
      this.dt = dt;

      registry = new YoVariableRegistry(getClass().getSimpleName() + side.getPascalCaseName());
      linePointA = new YoFramePoint2D("FootRotationPointA", ReferenceFrame.getWorldFrame(), registry);
      linePointB = new YoFramePoint2D("FootRotationPointB", ReferenceFrame.getWorldFrame(), registry);
      parentRegistry.addChild(registry);

      String feetManagerName = FeetManager.class.getSimpleName();
      String paramRegistryName = getClass().getSimpleName() + "Parameters";
      omegaThresholdForEstimation = ParameterProvider.getOrCreateParameter(feetManagerName, paramRegistryName, "omegaThresholdForEstimation", registry, 0.1);
      decayBreakFrequency = ParameterProvider.getOrCreateParameter(feetManagerName, paramRegistryName, "decayBreakFrequency", registry, 1.0);
      filterBreakFrequency = ParameterProvider.getOrCreateParameter(feetManagerName, paramRegistryName, "filterBreakFrequency", registry, 1.0);
      rotationThreshold = ParameterProvider.getOrCreateParameter(feetManagerName, paramRegistryName, "rotationThreshold", registry, 0.4);

      integratedRotationAngle = new YoDouble(side.getLowerCaseName() + "IntegratedRotationAngle", registry);
      isRotating = new YoBoolean(side.getLowerCaseName() + "IsRotating", registry);

      DoubleProvider alpha = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(filterBreakFrequency.getValue(), dt);
      filteredPointOfRotation = new AlphaFilteredYoFramePoint2d(side + "FilteredPointOfRotation", "", registry, alpha, soleFrame);
      filteredAxisOfRotation = new AlphaFilteredYoFrameVector2d(side + "FilteredAxisOfRotation", "", registry, alpha, soleFrame);

      reset();

      if (graphicsRegistry != null)
      {
         Artifact lineArtifact = new YoArtifactLineSegment2d(side.getLowerCaseName() + "LineOfRotation", linePointA, linePointB, Color.ORANGE, 0.005, 0.01);
         graphicsRegistry.registerArtifact(getClass().getSimpleName(), lineArtifact);
      }
   }

   private final FrameVector3D tempPointOfRotation = new FrameVector3D();

   public boolean compute()
   {
      // 1. Using the twist of the foot
      TwistReadOnly soleFrameTwist = soleFrame.getTwistOfFrame();
      double omegaSquared = soleFrameTwist.getAngularPart().lengthSquared();
      if (omegaSquared > omegaThresholdForEstimation.getValue())
      {
         tempPointOfRotation.setToZero(soleFrame);
         tempPointOfRotation.cross(soleFrameTwist.getAngularPart(), soleFrameTwist.getLinearPart());
         tempPointOfRotation.scale(1.0 / omegaSquared);
         lineOfRotationInSole.setToZero(soleFrame);
         lineOfRotationInSole.getPoint().set(tempPointOfRotation);
         lineOfRotationInSole.getDirection().set(soleFrameTwist.getAngularPart());

         double omega = lineOfRotationInSole.getDirection().length();
         integratedRotationAngle.add(dt * omega);

         lineOfRotationInSole.getDirection().scale(1.0 / omega);

         if (lineOfRotationInSole.getDirection().dot(filteredAxisOfRotation) < 0.0)
         {
            lineOfRotationInSole.getDirection().negate();
         }

         // Filter the line of rotation:
         filteredPointOfRotation.update(lineOfRotationInSole.getPoint());
         filteredAxisOfRotation.update(lineOfRotationInSole.getDirection());
         lineOfRotationInSole.set(filteredPointOfRotation, filteredAxisOfRotation);
         lineOfRotationInSole.getDirection().normalize();
      }

      if (!isRotating.getValue())
      {
         isRotating.set(integratedRotationAngle.getValue() > rotationThreshold.getValue());
      }
      integratedRotationAngle.mul(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(decayBreakFrequency.getValue(), dt));

      updateGraphics();

      return isRotating.getValue();
   }

   private final FrameLine3DBasics tempLineOfRotationInWorld = new FrameLine3D();

   private void updateGraphics()
   {
      if (!isRotating.getValue())
      {
         linePointA.setToNaN();
         linePointB.setToNaN();
         return;
      }

      tempLineOfRotationInWorld.setToZero(soleFrame);
      tempLineOfRotationInWorld.getPoint().set(lineOfRotationInSole.getPoint());
      tempLineOfRotationInWorld.getDirection().set(lineOfRotationInSole.getDirection());
      tempLineOfRotationInWorld.changeFrame(ReferenceFrame.getWorldFrame());

      linePointA.set(tempLineOfRotationInWorld.getDirection());
      linePointA.scale(-0.05);
      linePointA.add(tempLineOfRotationInWorld.getPointX(), tempLineOfRotationInWorld.getPointY());

      linePointB.set(tempLineOfRotationInWorld.getDirection());
      linePointB.scale(0.05);
      linePointB.add(tempLineOfRotationInWorld.getPointX(), tempLineOfRotationInWorld.getPointY());
   }

   public FrameLine2DReadOnly getLineOfRotation()
   {
      return lineOfRotationInSole;
   }

   public void reset()
   {
      linePointA.setToNaN();
      linePointB.setToNaN();
      filteredPointOfRotation.reset();
      filteredAxisOfRotation.reset();
      integratedRotationAngle.set(0.0);
      isRotating.set(false);
   }
}
