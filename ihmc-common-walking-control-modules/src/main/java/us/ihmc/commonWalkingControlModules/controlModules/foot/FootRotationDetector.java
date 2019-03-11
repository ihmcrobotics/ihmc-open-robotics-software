package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.awt.Color;

import us.ihmc.commonWalkingControlModules.momentumBasedController.ParameterProvider;
import us.ihmc.euclid.referenceFrame.FrameLine3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameLine2DBasics;
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
import us.ihmc.yoVariables.variable.YoFrameLine2D;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFrameVector2D;

/**
 * This class computes whether a foot is rotating.</br>
 * If that is the case provides an estimate of the line of rotation. This class does not rely on a center of pressure
 * estimate and can be used with robots that do not have force-torque sensing in the feet.
 * <p>
 * The strategy employed is to use the measured twist of the foot to compute the current line of rotation. This requires
 * the angular velocity of the foot to be sufficiently large. A threshold determines if that is the case. The speed of
 * rotation is the integrated using a leak rate. If the integral which is a measure of absolute foot rotation exceeds
 * a second threshold the foot is assumed to rotate.]
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

   private final FixedFrameLine2DBasics lineOfRotationInSole;
   private final YoDouble integratedRotationAngle;
   private final YoDouble absoluteFootOmega;
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
      omegaThresholdForEstimation = ParameterProvider.getOrCreateParameter(feetManagerName, paramRegistryName, "omegaThresholdForEstimation", registry, 3.0);
      decayBreakFrequency = ParameterProvider.getOrCreateParameter(feetManagerName, paramRegistryName, "decayBreakFrequency", registry, 1.0);
      filterBreakFrequency = ParameterProvider.getOrCreateParameter(feetManagerName, paramRegistryName, "filterBreakFrequency", registry, 1.0);
      rotationThreshold = ParameterProvider.getOrCreateParameter(feetManagerName, paramRegistryName, "rotationThreshold", registry, 0.2);

      integratedRotationAngle = new YoDouble(side.getLowerCaseName() + "IntegratedRotationAngle", registry);
      absoluteFootOmega = new YoDouble(side.getLowerCaseName() + "AbsoluteFootOmega", registry);
      isRotating = new YoBoolean(side.getLowerCaseName() + "IsRotating", registry);

      YoFramePoint2D point = new YoFramePoint2D(side.getLowerCaseName() + "LineOfRotationPoint", soleFrame, registry);
      YoFrameVector2D direction = new YoFrameVector2D(side.getLowerCaseName() + "LineOfRotationDirection", soleFrame, registry);
      lineOfRotationInSole = new YoFrameLine2D(point, direction);

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
      // Using the twist of the foot
      TwistReadOnly soleFrameTwist = soleFrame.getTwistOfFrame();
      double omegaSquared = soleFrameTwist.getAngularPart().lengthSquared();
      absoluteFootOmega.set(Math.sqrt(omegaSquared));
      if (absoluteFootOmega.getValue() > omegaThresholdForEstimation.getValue())
      {
         tempPointOfRotation.setToZero(soleFrame);
         tempPointOfRotation.cross(soleFrameTwist.getAngularPart(), soleFrameTwist.getLinearPart());
         tempPointOfRotation.scale(1.0 / omegaSquared);
         lineOfRotationInSole.setToZero();
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
      else if (!isRotating.getValue())
      {
         filteredPointOfRotation.reset();
         filteredAxisOfRotation.reset();
         lineOfRotationInSole.setToZero();
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
      lineOfRotationInSole.setToZero();
      integratedRotationAngle.set(0.0);
      absoluteFootOmega.set(0.0);
      isRotating.set(false);
   }
}
