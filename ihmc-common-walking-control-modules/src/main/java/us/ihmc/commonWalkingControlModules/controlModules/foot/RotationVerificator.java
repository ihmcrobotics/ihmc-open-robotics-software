package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * The purpose of this class is to check if it is probable that the foot is rotating, given a
 * line of rotation, the actual cop, and the desired cop.
 *
 * @author Georg
 *
 */
public class RotationVerificator
{
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry;

   private final ReferenceFrame soleFrame;
   private final YoFrameVector2D copError;

   /**
    * Check if the error between cop and desired cop perpendicular the line of
    * rotation is above a threshold
    */
   private final YoDouble perpendicularCopError;
   private final YoDouble perpendicularCopErrorThreshold;
   private final YoBoolean perpendicularCopErrorAboveThreshold;

   /**
    * Check if the angle between cop error vector and the perpendicular error
    * is below a threshold
    */
   private final YoDouble angleBetweenCopErrorAndLine;
   private final YoDouble angleThreshold;
   private final YoBoolean angleOkay;

   /**
    * Check if desired cop is in area that will be cut off
    */
   private final YoBoolean desiredCopOnCorrectSide;

   public RotationVerificator(String namePrefix, ReferenceFrame soleFrame, ExplorationParameters explorationParameters, YoRegistry parentRegistry)
   {
      this.soleFrame = soleFrame;

      registry = new YoRegistry(namePrefix + name);
      parentRegistry.addChild(registry);

      copError = new YoFrameVector2D(namePrefix + "CopError", "", soleFrame, registry);

      perpendicularCopError = new YoDouble(namePrefix + "PerpendicularCopError", registry);
      perpendicularCopErrorThreshold = explorationParameters.getPerpendicluarCopErrorThreshold();
      perpendicularCopErrorAboveThreshold = new YoBoolean(namePrefix + "PerpendicularCopErrorAboveThreshold", registry);

      angleBetweenCopErrorAndLine = new YoDouble(namePrefix + "AngleBetweenCopErrorAndLine", registry);
      angleThreshold = explorationParameters.getCopAllowedAreaOpeningAngle();
      angleOkay = new YoBoolean(namePrefix + "AngleOkay", registry);

      desiredCopOnCorrectSide = new YoBoolean(namePrefix + "DesiredCopOnCorrectSide", registry);
   }


   public boolean isRotating(FramePoint2DReadOnly measuredCoP, FramePoint2DReadOnly desiredCop, FrameLine2DReadOnly lineOfRotation)
   {
      lineOfRotation.checkReferenceFrameMatch(soleFrame);

      if (!lineOfRotation.isPointOnLine(measuredCoP))
         return false;

      copError.sub(desiredCop, measuredCoP);

      perpendicularCopError.set(lineOfRotation.distance(desiredCop));
      boolean errorAboveThreshold = perpendicularCopError.getDoubleValue() >= perpendicularCopErrorThreshold.getDoubleValue();
      perpendicularCopErrorAboveThreshold.set(errorAboveThreshold);

      double acos = perpendicularCopError.getDoubleValue() / copError.length();
      angleBetweenCopErrorAndLine.set(Math.acos(Math.abs(acos)));
      boolean angleInBounds = angleBetweenCopErrorAndLine.getDoubleValue() <= angleThreshold.getDoubleValue();
      angleOkay.set(angleInBounds);

      boolean correctSide = lineOfRotation.isPointOnRightSideOfLine(desiredCop);
      desiredCopOnCorrectSide.set(correctSide);

      return errorAboveThreshold && angleInBounds && correctSide;
   }
}
