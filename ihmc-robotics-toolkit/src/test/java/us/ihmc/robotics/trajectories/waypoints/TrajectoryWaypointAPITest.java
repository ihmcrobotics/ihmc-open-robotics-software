package us.ihmc.robotics.trajectories.waypoints;

import java.lang.reflect.Method;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import org.junit.Assert;
import org.junit.Test;

import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.geometry.interfaces.EuclideanWaypointInterface;
import us.ihmc.robotics.geometry.interfaces.FrameEuclideanWaypointInterface;
import us.ihmc.robotics.geometry.interfaces.FrameSE3WaypointInterface;
import us.ihmc.robotics.geometry.interfaces.FrameSO3WaypointInterface;
import us.ihmc.robotics.geometry.interfaces.SE3WaypointInterface;
import us.ihmc.robotics.geometry.interfaces.SO3WaypointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanTrajectoryPointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameEuclideanTrajectoryPointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSE3TrajectoryPointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSO3TrajectoryPointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3TrajectoryPointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3TrajectoryPointInterface;

public class TrajectoryWaypointAPITest
{
   private final static Map<Class<?>, Class<?>> framelessTypesToFrameTypesTable;
   static
   {
      HashMap<Class<?>, Class<?>> modifiableMap = new HashMap<>();

      modifiableMap.put(EuclideanWaypointInterface.class, FrameEuclideanWaypointInterface.class);
      modifiableMap.put(SO3WaypointInterface.class, FrameSO3WaypointInterface.class);
      modifiableMap.put(SE3WaypointInterface.class, FrameSE3WaypointInterface.class);

      modifiableMap.put(EuclideanTrajectoryPointInterface.class, FrameEuclideanTrajectoryPointInterface.class);
      modifiableMap.put(SO3TrajectoryPointInterface.class, FrameSO3TrajectoryPointInterface.class);
      modifiableMap.put(SE3TrajectoryPointInterface.class, FrameSE3TrajectoryPointInterface.class);

      framelessTypesToFrameTypesTable = Collections.unmodifiableMap(modifiableMap);
   }

   private final static Map<Class<?>, Class<?>> waypointTypesToTrajectoryTypesTable;
   static
   {
      HashMap<Class<?>, Class<?>> modifiableMap = new HashMap<>();

      modifiableMap.put(EuclideanWaypointInterface.class, EuclideanTrajectoryPointInterface.class);
      modifiableMap.put(SO3WaypointInterface.class, SO3TrajectoryPointInterface.class);
      modifiableMap.put(SE3WaypointInterface.class, SE3TrajectoryPointInterface.class);

      modifiableMap.put(FrameEuclideanWaypointInterface.class, FrameEuclideanTrajectoryPointInterface.class);
      modifiableMap.put(FrameSO3WaypointInterface.class, FrameSO3TrajectoryPointInterface.class);
      modifiableMap.put(FrameSE3WaypointInterface.class, FrameSE3TrajectoryPointInterface.class);

      waypointTypesToTrajectoryTypesTable = Collections.unmodifiableMap(modifiableMap);
   }

   private final static Map<Class<?>, Class<?>> argumentTypeFramelessToFrameMap;
   static
   {
      HashMap<Class<?>, Class<?>> modifiableMap = new HashMap<>();

      modifiableMap.put(EuclideanWaypointInterface.class, FrameEuclideanWaypointInterface.class);
      modifiableMap.put(SO3WaypointInterface.class, FrameSO3WaypointInterface.class);
      modifiableMap.put(SE3WaypointInterface.class, FrameSE3WaypointInterface.class);

      modifiableMap.put(EuclideanTrajectoryPointInterface.class, FrameEuclideanTrajectoryPointInterface.class);
      modifiableMap.put(SO3TrajectoryPointInterface.class, FrameSO3TrajectoryPointInterface.class);
      modifiableMap.put(SE3TrajectoryPointInterface.class, FrameSE3TrajectoryPointInterface.class);

      modifiableMap.put(Point3DReadOnly.class, FramePoint3DReadOnly.class);
      modifiableMap.put(QuaternionReadOnly.class, FrameQuaternionReadOnly.class);
      modifiableMap.put(Vector3DReadOnly.class, FrameVector3DReadOnly.class);
      modifiableMap.put(Pose3DReadOnly.class, FramePose3DReadOnly.class);

      modifiableMap.put(Point3DBasics.class, FixedFramePoint3DBasics.class);
      modifiableMap.put(QuaternionBasics.class, FixedFrameQuaternionBasics.class);
      modifiableMap.put(Vector3DBasics.class, FixedFrameVector3DBasics.class);
      modifiableMap.put(Pose3DBasics.class, FixedFramePose3DBasics.class);

      // unchanged:
      modifiableMap.put(double.class, double.class);
      modifiableMap.put(Transform.class, Transform.class);

      argumentTypeFramelessToFrameMap = Collections.unmodifiableMap(modifiableMap);
   }

   private final static Map<Class<?>, Class<?>> returnTypeFramelessToFrameMap;
   static
   {
      HashMap<Class<?>, Class<?>> modifiableMap = new HashMap<>();

      modifiableMap.put(Point3DReadOnly.class, FramePoint3DReadOnly.class);
      modifiableMap.put(QuaternionReadOnly.class, FrameQuaternionReadOnly.class);
      modifiableMap.put(Vector3DReadOnly.class, FrameVector3DReadOnly.class);
      modifiableMap.put(Pose3DReadOnly.class, FramePose3DReadOnly.class);

      modifiableMap.put(Point3DBasics.class, FramePoint3DBasics.class);
      modifiableMap.put(QuaternionBasics.class, FrameQuaternionBasics.class);
      modifiableMap.put(Vector3DBasics.class, FrameVector3DBasics.class);
      modifiableMap.put(Pose3DBasics.class, FramePose3DBasics.class);

      // unchanged:
      modifiableMap.put(double.class, double.class);
      modifiableMap.put(boolean.class, boolean.class);

      returnTypeFramelessToFrameMap = Collections.unmodifiableMap(modifiableMap);
   }

   private final static Map<Class<?>, Class<?>> waypointToTrajectoryPointMap;
   static
   {
      HashMap<Class<?>, Class<?>> modifiableMap = new HashMap<>();

      modifiableMap.put(EuclideanWaypointInterface.class, EuclideanTrajectoryPointInterface.class);
      modifiableMap.put(SO3WaypointInterface.class, SO3TrajectoryPointInterface.class);
      modifiableMap.put(SE3WaypointInterface.class, SE3TrajectoryPointInterface.class);

      modifiableMap.put(FrameEuclideanWaypointInterface.class, FrameEuclideanTrajectoryPointInterface.class);
      modifiableMap.put(FrameSO3WaypointInterface.class, FrameSO3TrajectoryPointInterface.class);
      modifiableMap.put(FrameSE3WaypointInterface.class, FrameSE3TrajectoryPointInterface.class);

      // unchanged:
      modifiableMap.put(Point3DReadOnly.class, Point3DReadOnly.class);
      modifiableMap.put(QuaternionReadOnly.class, QuaternionReadOnly.class);
      modifiableMap.put(Vector3DReadOnly.class, Vector3DReadOnly.class);
      modifiableMap.put(Pose3DReadOnly.class, Pose3DReadOnly.class);

      modifiableMap.put(Point3DBasics.class, Point3DBasics.class);
      modifiableMap.put(QuaternionBasics.class, QuaternionBasics.class);
      modifiableMap.put(Vector3DBasics.class, Vector3DBasics.class);
      modifiableMap.put(Pose3DBasics.class, Pose3DBasics.class);

      modifiableMap.put(FramePoint3DBasics.class, FramePoint3DBasics.class);
      modifiableMap.put(FrameQuaternionBasics.class, FrameQuaternionBasics.class);
      modifiableMap.put(FrameVector3DBasics.class, FrameVector3DBasics.class);
      modifiableMap.put(FramePose3DBasics.class, FramePose3DBasics.class);

      modifiableMap.put(FixedFramePoint3DBasics.class, FixedFramePoint3DBasics.class);
      modifiableMap.put(FixedFrameQuaternionBasics.class, FixedFrameQuaternionBasics.class);
      modifiableMap.put(FixedFrameVector3DBasics.class, FixedFrameVector3DBasics.class);
      modifiableMap.put(FixedFramePose3DBasics.class, FixedFramePose3DBasics.class);

      modifiableMap.put(FramePoint3DReadOnly.class, FramePoint3DReadOnly.class);
      modifiableMap.put(FrameQuaternionReadOnly.class, FrameQuaternionReadOnly.class);
      modifiableMap.put(FrameVector3DReadOnly.class, FrameVector3DReadOnly.class);
      modifiableMap.put(FramePose3DReadOnly.class, FramePose3DReadOnly.class);

      modifiableMap.put(double.class, double.class);
      modifiableMap.put(boolean.class, boolean.class);
      modifiableMap.put(Transform.class, Transform.class);
      modifiableMap.put(ReferenceFrame.class, ReferenceFrame.class);
      modifiableMap.put(ReferenceFrameHolder.class, ReferenceFrameHolder.class);

      waypointToTrajectoryPointMap = Collections.unmodifiableMap(modifiableMap);
   }

   private final static Set<String> geometryMethodsToIgnore;
   static
   {
      Set<String> modifiableSet = new HashSet<>();

      modifiableSet.add("geometricallyEquals");
      modifiableSet.add("positionDistance");
      modifiableSet.add("orientationDistance");

      geometryMethodsToIgnore = Collections.unmodifiableSet(modifiableSet);
   }

   @Test(timeout = 3000)
   public void testFrameOverwrites()
   {
      framelessTypesToFrameTypesTable.forEach((framelessClass, frameClass) -> {
         assertOverloadingWithFrameObjects(frameClass, framelessClass);
      });
   }

   @Test(timeout = 3000)
   public void testTrajectoryOverwrites()
   {
      waypointTypesToTrajectoryTypesTable.forEach((framelessClass, frameClass) -> {
         assertOverloadingWithTrajectoryObjects(frameClass, framelessClass);
      });
   }

   private static void assertOverloadingWithFrameObjects(Class<?> frameClass, Class<?> framelessClass)
   {
      Arrays.asList(framelessClass.getMethods()).forEach(method -> {
         Class<?>[] expectedSignature = createExpectedSignature(method, argumentTypeFramelessToFrameMap);
         Class<?> expectedReturn = createExpectedReturn(method, returnTypeFramelessToFrameMap);
         asertMethodExists(frameClass, method.getName(), expectedSignature, expectedReturn);
      });
   }

   private static void assertOverloadingWithTrajectoryObjects(Class<?> frameClass, Class<?> framelessClass)
   {
      Arrays.asList(framelessClass.getMethods()).forEach(method -> {
         if (geometryMethodsToIgnore.contains(method.getName()))
         {
            return;
         }

         Class<?>[] expectedSignature = createExpectedSignature(method, waypointToTrajectoryPointMap);
         Class<?> expectedReturn = createExpectedReturn(method, waypointToTrajectoryPointMap);
         asertMethodExists(frameClass, method.getName(), expectedSignature, expectedReturn);
      });
   }

   private static void asertMethodExists(Class<?> clazz, String methodName, Class<?>[] signature, Class<?> returnType)
   {
      boolean foundMatch = Arrays.asList(clazz.getMethods()).stream().filter(method -> {
         if (!method.getName().equals(methodName))
         {
            return false;
         }
         Class<?>[] methodSignature = method.getParameterTypes();
         if (signature.length != methodSignature.length)
         {
            return false;
         }
         for (int i = 0; i < signature.length; i++)
         {
            if (signature[i] != methodSignature[i])
            {
               return false;
            }
         }
         if (method.getReturnType() != returnType)
         {
            return false;
         }
         return true;
      }).findFirst().isPresent();

      String methodString = createMethodString(methodName, signature, returnType);
      Assert.assertTrue("Did not find method \n" + methodString + "\nIn class " + clazz.getSimpleName(), foundMatch);
   }

   private static String createMethodString(String methodName, Class<?>[] signature, Class<?> returnType)
   {
      String ret = returnType.getSimpleName() + " " + methodName + "(";
      for (int i = 0; i < signature.length; i++)
      {
         ret += signature[i].getSimpleName();
         if (i < signature.length - 1)
         {
            ret += ", ";
         }
      }
      ret += ")";
      return ret;
   }

   private static Class<?>[] createExpectedSignature(Method method, Map<Class<?>, Class<?>> argumentTypeMap)
   {
      Class<?>[] parameterTypes = method.getParameterTypes();
      Class<?>[] ret = new Class<?>[parameterTypes.length];

      for (int i = 0; i < parameterTypes.length; i++)
      {
         Class<?> parameterType = parameterTypes[i];
         if (!argumentTypeMap.containsKey(parameterType))
         {
            throw new RuntimeException("Please add a matching type for " + parameterType.getSimpleName() + " to argument types.");
         }
         ret[i] = argumentTypeMap.get(parameterType);
      }

      return ret;
   }

   private static Class<?> createExpectedReturn(Method method, Map<Class<?>, Class<?>> returnTypeMap)
   {
      Class<?> returnType = method.getReturnType();
      if (returnType == void.class)
      {
         return void.class;
      }
      if (!returnTypeMap.containsKey(returnType))
      {
         throw new RuntimeException("Please add a matching type for " + returnType.getSimpleName() + " to return types.");
      }
      return returnTypeMap.get(returnType);
   }
}
