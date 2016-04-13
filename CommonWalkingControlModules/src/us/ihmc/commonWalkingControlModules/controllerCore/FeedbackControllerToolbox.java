package us.ihmc.commonWalkingControlModules.controllerCore;

import java.util.EnumMap;
import java.util.HashMap;
import java.util.Map;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public class FeedbackControllerToolbox
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public enum Type
   {
      DESIRED("Desired"),
      CURRENT("Current"),
      FEEDFORWARD("FeedForward"),
      ACHIEVED("Achieved"),
      ERROR("Error");

      private final String name;

      private Type(String name)
      {
         this.name = name;
      }

      public String getName()
      {
         return name;
      }
   };

   public enum Space
   {
      POSITION("Position"),
      ORIENTATION("Orientation"),
      ROTATION_VECTOR("RotationVector"),
      LINEAR_VELOCITY("LinearVelocity"),
      ANGULAR_VELOCITY("AngularVelocity"),
      LINEAR_ACCELERATION("LinearAcceleration"),
      ANGULAR_ACCELERATION("AngularAcceleration");

      private final String name;

      private Space(String name)
      {
         this.name = name;
      }

      public String getName()
      {
         return name;
      }

      @Override
      public String toString()
      {
         return name;
      }
   }

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final Map<RigidBody, EnumMap<Type, YoFramePoint>> endEffectorPositions = new HashMap<>();
   private final Map<RigidBody, EnumMap<Type, YoFrameQuaternion>> endEffectorOrientations = new HashMap<>();
   private final Map<RigidBody, EnumMap<Type, EnumMap<Space, YoFrameVector>>> endEffectorDataVectors = new HashMap<>();

   public FeedbackControllerToolbox(YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
   }

   public YoFramePoint getOrCreatePosition(RigidBody endEffector, Type type)
   {
      EnumMap<Type, YoFramePoint> typeDependentPositions = endEffectorPositions.get(endEffector);

      if (typeDependentPositions == null)
      {
         typeDependentPositions = new EnumMap<>(Type.class);
         endEffectorPositions.put(endEffector, typeDependentPositions);
      }

      YoFramePoint yoFramePoint = typeDependentPositions.get(type);

      if (yoFramePoint == null)
      {
         String namePrefix = endEffector.getName();
         namePrefix += type.getName();
         namePrefix += Space.POSITION.getName();
         yoFramePoint = new YoFramePoint(namePrefix, worldFrame, registry);
         typeDependentPositions.put(type, yoFramePoint);
      }

      return yoFramePoint;
   }

   public YoFrameQuaternion getOrCreateOrientation(RigidBody endEffector, Type type)
   {
      EnumMap<Type, YoFrameQuaternion> typeDependentOrientations = endEffectorOrientations.get(endEffector);

      if (typeDependentOrientations == null)
      {
         typeDependentOrientations = new EnumMap<>(Type.class);
         endEffectorOrientations.put(endEffector, typeDependentOrientations);
      }

      YoFrameQuaternion yoFrameQuaternion = typeDependentOrientations.get(type);

      if (yoFrameQuaternion == null)
      {
         String namePrefix = endEffector.getName();
         namePrefix += type.getName();
         namePrefix += Space.ORIENTATION.getName();
         yoFrameQuaternion = new YoFrameQuaternion(namePrefix, worldFrame, registry);
         typeDependentOrientations.put(type, yoFrameQuaternion);
      }

      return yoFrameQuaternion;
   }

   public YoFrameVector getOrCreateDataVector(RigidBody endEffector, Type type, Space space)
   {
      EnumMap<Type, EnumMap<Space, YoFrameVector>> dataVectorStep1 = endEffectorDataVectors.get(endEffector);

      if (dataVectorStep1 == null)
      {
         dataVectorStep1 = new EnumMap<>(Type.class);
         endEffectorDataVectors.put(endEffector, dataVectorStep1);
      }

      EnumMap<Space, YoFrameVector> dataVectorStep2 = dataVectorStep1.get(type);

      if (dataVectorStep2 == null)
      {
         dataVectorStep2 = new EnumMap<>(Space.class);
         dataVectorStep1.put(type, dataVectorStep2);
      }

      YoFrameVector yoFrameVector = dataVectorStep2.get(space);

      if (yoFrameVector == null)
      {
         String namePrefix = endEffector.getName();
         namePrefix += type.getName();
         namePrefix += space.getName();
         yoFrameVector = new YoFrameVector(namePrefix, worldFrame, registry);
         dataVectorStep2.put(space, yoFrameVector);
      }

      return yoFrameVector;
   }
}
