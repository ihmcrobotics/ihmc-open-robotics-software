package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.EnumMap;

public class CombinedFootRotationDetector implements FootRotationDetector
{
   private enum RotationDetectorType
   {
      GEOMETRIC, KINEMATIC, VELOCITY, ANY, KINEMATIC_AND_VELOCITY;

      static final RotationDetectorType[] values = {/*GEOMETRIC,*/ KINEMATIC, VELOCITY};
   }

   private final YoEnum<RotationDetectorType> rotationDetectorType;
   private final EnumMap<RotationDetectorType, FootRotationDetector> rotationDetectors = new EnumMap<>(RotationDetectorType.class);

   private final YoBoolean isRotating;


   public CombinedFootRotationDetector(RobotSide side,
                                       MovingReferenceFrame soleFrame,
                                       FootholdRotationParameters rotationParameters,
                                       double dt,
                                       YoVariableRegistry registry)
   {
      FootRotationDetector velocityRotationDetector = new VelocityFootRotationDetector(side, soleFrame, rotationParameters, dt, registry);
      FootRotationDetector kinematicRotationDetector = new KinematicFootRotationDetector(side, soleFrame, rotationParameters, dt, registry);
      rotationDetectors.put(RotationDetectorType.KINEMATIC, kinematicRotationDetector);
      rotationDetectors.put(RotationDetectorType.VELOCITY, velocityRotationDetector);

      rotationDetectorType = YoEnum.create(side.getCamelCaseName() + "RotationDetectorType", RotationDetectorType.class, registry);

      rotationDetectorType.set(RotationDetectorType.KINEMATIC_AND_VELOCITY);

      isRotating = new YoBoolean(side.getLowerCaseName() + "IsRotating", registry);
   }

   @Override
   public void reset()
   {
      isRotating.set(false);

      for (RotationDetectorType type : RotationDetectorType.values)
         rotationDetectors.get(type).reset();
   }

   @Override
   public boolean compute()
   {
      boolean rotationDetected;
      if (rotationDetectorType.getEnumValue() == RotationDetectorType.ANY)
      {
         rotationDetected = false;
         for (RotationDetectorType type : RotationDetectorType.values)
         {
            if (rotationDetectors.get(type).compute())
               rotationDetected = true;
         }
      }
      else if (rotationDetectorType.getEnumValue() == RotationDetectorType.KINEMATIC_AND_VELOCITY)
      {
         rotationDetected = rotationDetectors.get(RotationDetectorType.KINEMATIC).compute();
         rotationDetected |= rotationDetectors.get(RotationDetectorType.VELOCITY).compute();
      }
      else
      {

         rotationDetected = rotationDetectors.get(rotationDetectorType.getEnumValue()).compute();
      }

      isRotating.set(rotationDetected);
      return rotationDetected;
   }

   @Override
   public boolean isRotating()
   {
      return isRotating.getBooleanValue();
   }
}
