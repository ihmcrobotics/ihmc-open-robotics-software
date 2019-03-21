package us.ihmc.commonWalkingControlModules.controllerCore.command;

import static org.junit.jupiter.api.Assertions.fail;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Random;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.ejml.data.DenseMatrix64F;
import org.junit.jupiter.api.Test;
import org.opentest4j.AssertionFailedError;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualEffortCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommandBuffer;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FrameMatrix3D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.lists.DenseMatrixArrayList;
import us.ihmc.robotics.lists.FrameTupleArrayList;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;

public class ControllerCoreCommandCodeQualityTest
{
   /**
    * Assert that all the commands have an empty constructor.
    * 
    * @throws Exception
    */
   @SuppressWarnings("rawtypes")
   @Test
   void testEmptyConstructor()
   {
      String errorMessage = "";

      for (Class<? extends InverseDynamicsCommand> commandType : ControllerCoreCommandRandomTools.getInverseDynamicsCommandTypes())
      {
         errorMessage += testEmptyConstructor(commandType);
      }

      for (Class<? extends InverseKinematicsCommand> commandType : ControllerCoreCommandRandomTools.getInverseKinematicsCommandTypes())
      {
         errorMessage += testEmptyConstructor(commandType);
      }

      for (Class<? extends VirtualModelControlCommand> commandType : ControllerCoreCommandRandomTools.getVirtualModelControlCommandTypes())
      {
         errorMessage += testEmptyConstructor(commandType);
      }

      for (Class<? extends FeedbackControlCommand> commandType : ControllerCoreCommandRandomTools.getFeedbackControlCommandTypes())
      {
         errorMessage += testEmptyConstructor(commandType);
      }

      if (!errorMessage.isEmpty())
         fail("The following issues were detected:\n" + errorMessage);
   }

   private static String testEmptyConstructor(Class<?> clazz)
   {
      if (clazz.isInterface())
         return "";

      Constructor<?> emptyConstructor;
      try
      {
         emptyConstructor = clazz.getDeclaredConstructor();
      }
      catch (NoSuchMethodException | SecurityException e)
      {
         return "Encountered problem with: " + clazz.getSimpleName() + " " + e.getMessage() + "\n";
      }

      try
      {
         emptyConstructor.newInstance();
      }
      catch (InstantiationException | IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
      {
         return "Encountered problem when invoking empty constructor for: " + clazz.getSimpleName() + " " + e.getMessage() + "\n";
      }
      return "";
   }

   @SuppressWarnings("rawtypes")
   @Test
   void testEqualsWithEmptyObjects() throws Exception
   {
      String errorMessage = "";

      for (Class<? extends InverseDynamicsCommand> commandType : ControllerCoreCommandRandomTools.getInverseDynamicsCommandTypes())
      {
         errorMessage += testEqualsWithEmptyObject(commandType);
      }

      for (Class<? extends InverseKinematicsCommand> commandType : ControllerCoreCommandRandomTools.getInverseKinematicsCommandTypes())
      {
         errorMessage += testEqualsWithEmptyObject(commandType);
      }

      for (Class<? extends VirtualModelControlCommand> commandType : ControllerCoreCommandRandomTools.getVirtualModelControlCommandTypes())
      {
         errorMessage += testEqualsWithEmptyObject(commandType);
      }

      for (Class<? extends FeedbackControlCommand> commandType : ControllerCoreCommandRandomTools.getFeedbackControlCommandTypes())
      {
         errorMessage += testEqualsWithEmptyObject(commandType);
      }

      if (!errorMessage.isEmpty())
         fail("The following issues were detected:\n" + errorMessage);
   }

   @Test
   void testEqualsWithRandomObjects() throws Exception
   {
      Random random = new Random(4534);
      boolean verbose = false;

      List<JointBasics> joints = MultiBodySystemRandomTools.nextJointTree(random, 20);
      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
      List<ReferenceFrame> referenceFramesList = rootBody.subtreeStream().map(RigidBodyBasics::getBodyFixedFrame).collect(Collectors.toList());
      SubtreeStreams.fromChildren(rootBody).map(JointBasics::getFrameBeforeJoint).forEach(referenceFramesList::add);
      SubtreeStreams.fromChildren(rootBody).map(JointBasics::getFrameAfterJoint).forEach(referenceFramesList::add);
      ReferenceFrame[] referenceFrames = referenceFramesList.toArray(new ReferenceFrame[0]);

      Set<Field> fieldsToIgnore = new HashSet<>();
      // The following are fields used for internal computation but do not reflect the state of the object.
      fieldsToIgnore.add(DefaultPID3DGains.class.getDeclaredField("dampingRatios"));
      fieldsToIgnore.add(PDGains.class.getDeclaredField("zeta"));
      fieldsToIgnore.add(SelectionMatrix3D.class.getDeclaredField("frameMatrix"));
      fieldsToIgnore.add(WeightMatrix3D.class.getDeclaredField("frameMatrix"));

      // Low-level types or types from 3rd party libraries assumed to be safe.
      Set<Class<?>> safeTypes = safeTypes();

      Set<Class<?>> allCommandTypes = new HashSet<>();
      allCommandTypes.addAll(ControllerCoreCommandRandomTools.getInverseDynamicsCommandTypes(InverseDynamicsCommandBuffer.class));
      allCommandTypes.addAll(ControllerCoreCommandRandomTools.getInverseKinematicsCommandTypes(InverseKinematicsCommandBuffer.class));
      allCommandTypes.addAll(ControllerCoreCommandRandomTools.getVirtualModelControlCommandTypes(VirtualModelControlCommandBuffer.class,
                                                                                                 VirtualEffortCommand.class));
      allCommandTypes.addAll(ControllerCoreCommandRandomTools.getFeedbackControlCommandTypes(FeedbackControlCommandBuffer.class));

      String errorMessage = "";

      for (Class<?> typeToTest : collectTypesAndSubTypes(allCommandTypes, safeTypes))
      {
         if (typeToTest.isInterface() || safeTypes.contains(typeToTest))
         {
            if (verbose)
               LogTools.info("Skipping: " + typeToTest.getSimpleName());
            continue;
         }

         for (Field field : typeToTest.getDeclaredFields())
         {
            if (Modifier.isStatic(field.getModifiers()) || fieldsToIgnore.contains(field))
            {
               if (verbose)
                  LogTools.info("Skipping: " + typeToTest.getSimpleName() + "." + field.getName());
               continue;
            }

            if (verbose)
               LogTools.info("Testing : " + typeToTest.getSimpleName() + "." + field.getName());

            boolean hasEqualsFailed = false;

            for (int attempt = 0; attempt < 5; attempt++)
            {
               long seed = random.nextLong();
               Object typeRandomInstance = ControllerCoreCommandRandomTools.nextTypeInstance(typeToTest, new Random(seed), true, rootBody, referenceFrames);
               Object typeRandomInstanceDuplicate = ControllerCoreCommandRandomTools.nextTypeInstance(typeToTest, new Random(seed), true, rootBody,
                                                                                                      referenceFrames);

               try
               {
                  ControllerCoreCommandRandomTools.randomizeField(random, field, typeRandomInstance, rootBody, referenceFrames);
               }
               catch (UnsupportedOperationException e)
               {
                  throw new AssertionFailedError("Encountered issue when randomizing: " + typeToTest.getSimpleName() + "." + field.getName()
                        + ". Probably missing a random generator for the type: " + field.getType(), e);
               }

               if (!typeRandomInstance.equals(typeRandomInstanceDuplicate))
               {
                  hasEqualsFailed = true;
                  break;
               }
            }

            if (!hasEqualsFailed)
            {
               errorMessage += typeToTest.getSimpleName() + ".equals(Object) seems to be missing test for the field: " + field.getName() + "\n";
            }
         }
      }

      if (!errorMessage.isEmpty())
         fail("The following issues were detected:\n" + errorMessage);
   }

   private static String testEqualsWithEmptyObject(Class<?> clazz) throws InstantiationException, IllegalAccessException
   {
      if (clazz.isInterface())
         return "";

      Object instance1 = clazz.newInstance();
      if (instance1.equals(null))
         return clazz.getSimpleName() + ".equals(null) returned true.\n";
      if (!instance1.equals(instance1))
         return "A fresh new instance of: " + clazz.getSimpleName() + " should be equal to itself.\n";

      Object instance2 = clazz.newInstance();
      if (!instance1.equals(instance2))
         return "Two fresh new instances of a " + clazz.getSimpleName() + " should be equal.\n";
      return "";
   }

   @Test
   void testRandomGeneratorsAPI() throws Exception
   {
      Set<Class<?>> typesToIgnore = new HashSet<>();
      typesToIgnore.add(ReferenceFrame.class);
      typesToIgnore.add(JointBasics.class);
      typesToIgnore.add(OneDoFJointBasics.class);
      typesToIgnore.add(RigidBodyBasics.class);
      // Used in WeightMatrix3D for computation only
      typesToIgnore.add(FrameMatrix3D.class);
      typesToIgnore.add(double[].class);

      Set<Class<?>> allCommandTypes = new HashSet<>();
      allCommandTypes.addAll(ControllerCoreCommandRandomTools.getInverseDynamicsCommandTypes(InverseDynamicsCommandBuffer.class));
      allCommandTypes.addAll(ControllerCoreCommandRandomTools.getInverseKinematicsCommandTypes(InverseKinematicsCommandBuffer.class));
      allCommandTypes.addAll(ControllerCoreCommandRandomTools.getVirtualModelControlCommandTypes(VirtualModelControlCommandBuffer.class,
                                                                                                 VirtualEffortCommand.class));
      allCommandTypes.addAll(ControllerCoreCommandRandomTools.getFeedbackControlCommandTypes(FeedbackControlCommandBuffer.class));
      allCommandTypes.removeIf(type -> type.isInterface());
      Map<Class<?>, Class<?>> typesToVerify = new HashMap<>();
      for (Class<?> commandType : allCommandTypes)
         typesToVerify.put(commandType, null);
      allCommandTypes.forEach(commandType -> extractTypeSubTypes(commandType, typesToVerify, safeTypes()));

      String errorMessage = "";

      Set<String> randomGeneratorNames = Stream.of(ControllerCoreCommandRandomTools.class.getDeclaredMethods()).map(Method::getName)
                                               .collect(Collectors.toSet());

      for (Entry<Class<?>, Class<?>> typeToVerify : typesToVerify.entrySet())
      {
         if (typeToVerify.getKey().isInterface() || typesToIgnore.contains(typeToVerify.getKey()))
            continue;

         if (!randomGeneratorNames.remove("next" + typeToVerify.getKey().getSimpleName())
               && !randomGeneratorNames.remove("randomize" + typeToVerify.getKey().getSimpleName()))
         {
            errorMessage += "Missing random generator for: " + typeToVerify.getKey().getSimpleName();
            if (typeToVerify.getValue() != null)
               errorMessage += " (discovered in type: " + typeToVerify.getValue().getSimpleName() + ")";
            errorMessage += "\n";
         }
      }

      if (!errorMessage.isEmpty())
         fail("The following issues were detected:\n" + errorMessage);
   }

   @Test
   void testRandomGeneratorsQuality() throws Exception
   {
      Random random = new Random(4534);
      boolean verbose = false;

      List<JointBasics> joints = MultiBodySystemRandomTools.nextJointTree(random, 20);
      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
      List<ReferenceFrame> referenceFramesList = rootBody.subtreeStream().map(RigidBodyBasics::getBodyFixedFrame).collect(Collectors.toList());
      SubtreeStreams.fromChildren(rootBody).map(JointBasics::getFrameBeforeJoint).forEach(referenceFramesList::add);
      SubtreeStreams.fromChildren(rootBody).map(JointBasics::getFrameAfterJoint).forEach(referenceFramesList::add);
      ReferenceFrame[] referenceFrames = referenceFramesList.toArray(new ReferenceFrame[0]);

      Set<Field> fieldsToIgnore = new HashSet<>();
      // The following are fields used for internal computation but do not reflect the state of the object.
      fieldsToIgnore.add(SelectionMatrix3D.class.getDeclaredField("frameMatrix"));
      fieldsToIgnore.add(WeightMatrix3D.class.getDeclaredField("frameMatrix"));

      // Low-level types or types from 3rd party libraries assumed to be safe.
      Set<Class<?>> safeTypes = safeTypes();

      Set<Class<?>> allCommandTypes = new HashSet<>();
      allCommandTypes.addAll(ControllerCoreCommandRandomTools.getInverseDynamicsCommandTypes(InverseDynamicsCommandBuffer.class));
      allCommandTypes.addAll(ControllerCoreCommandRandomTools.getInverseKinematicsCommandTypes(InverseKinematicsCommandBuffer.class));
      allCommandTypes.addAll(ControllerCoreCommandRandomTools.getVirtualModelControlCommandTypes(VirtualModelControlCommandBuffer.class,
                                                                                                 VirtualEffortCommand.class));
      allCommandTypes.addAll(ControllerCoreCommandRandomTools.getFeedbackControlCommandTypes(FeedbackControlCommandBuffer.class));

      String errorMessage = "";

      for (Class<?> typeToTest : collectTypesAndSubTypes(allCommandTypes, safeTypes))
      {
         if (typeToTest.isInterface() || safeTypes.contains(typeToTest))
         {
            if (verbose)
               LogTools.info("Skipping: " + typeToTest.getSimpleName());
            continue;
         }

         Object typeDefaultInstance = typeToTest.newInstance();

         for (Field field : typeToTest.getDeclaredFields())
         {
            if (Modifier.isStatic(field.getModifiers()) || fieldsToIgnore.contains(field))
            {
               if (verbose)
                  LogTools.info("Skipping: " + typeToTest.getSimpleName() + "." + field.getName());
               continue;
            }

            if (verbose)
               LogTools.info("Testing : " + typeToTest.getSimpleName() + "." + field.getName());

            boolean wasFieldDifferent = false;

            field.setAccessible(true);
            Object fieldTypeDefaultInstance = field.get(typeDefaultInstance);

            // We first assert that the random object is different from the instance created with empty constructor.
            for (int attempt = 0; attempt < 5; attempt++)
            {
               Object typeRandomInstance = ControllerCoreCommandRandomTools.nextTypeInstance(typeToTest, random, true, rootBody, referenceFrames);
               Object fieldTypeRandomInstance = field.get(typeRandomInstance);

               if (fieldTypeRandomInstance != null && !fieldTypeRandomInstance.equals(fieldTypeDefaultInstance))
               {
                  wasFieldDifferent = true;
                  break;
               }
            }

            if (!wasFieldDifferent)
            {
               errorMessage += "Random generator for " + typeToTest.getSimpleName() + " seems to be missing generation of the field: " + field.getName() + "\n";
            }

            // We now assert that 2 successive random objects have different values for each field.
            wasFieldDifferent = false;

            for (int attempt = 0; attempt < 10; attempt++)
            {
               Object typeRandomInstanceA = ControllerCoreCommandRandomTools.nextTypeInstance(typeToTest, random, true, rootBody, referenceFrames);
               Object fieldTypeRandomInstanceA = field.get(typeRandomInstanceA);
               Object typeRandomInstanceB = ControllerCoreCommandRandomTools.nextTypeInstance(typeToTest, random, true, rootBody, referenceFrames);
               Object fieldTypeRandomInstanceB = field.get(typeRandomInstanceB);

               if (fieldTypeRandomInstanceA == fieldTypeRandomInstanceB)
                  continue;
               // Some fields are optional and can be null sometimes, because of the previous test, we know it is not always null so it is fine.
               if (fieldTypeRandomInstanceA == null || !fieldTypeRandomInstanceA.equals(fieldTypeRandomInstanceB))
               {
                  wasFieldDifferent = true;
                  break;
               }
            }

            if (!wasFieldDifferent)
            {
               errorMessage += "Random generator for " + typeToTest.getSimpleName() + " seems to always generate the same value for the field: "
                     + field.getName() + "\n";
            }
         }
      }

      if (!errorMessage.isEmpty())
         fail("The following issues were detected:\n" + errorMessage);
   }

   @Test
   void testCommandCopySetterQuality() throws Exception
   {
      Random random = new Random(4534);
      boolean verbose = false;

      List<JointBasics> joints = MultiBodySystemRandomTools.nextJointTree(random, 20);
      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
      List<ReferenceFrame> referenceFramesList = rootBody.subtreeStream().map(RigidBodyBasics::getBodyFixedFrame).collect(Collectors.toList());
      SubtreeStreams.fromChildren(rootBody).map(JointBasics::getFrameBeforeJoint).forEach(referenceFramesList::add);
      SubtreeStreams.fromChildren(rootBody).map(JointBasics::getFrameAfterJoint).forEach(referenceFramesList::add);
      ReferenceFrame[] referenceFrames = referenceFramesList.toArray(new ReferenceFrame[0]);

      Set<Field> fieldsToIgnore = new HashSet<>();
      // The following are fields used for internal computation but do not reflect the state of the object.
      fieldsToIgnore.add(SelectionMatrix3D.class.getDeclaredField("frameMatrix"));
      fieldsToIgnore.add(WeightMatrix3D.class.getDeclaredField("frameMatrix"));

      // Low-level types or types from 3rd party libraries assumed to be safe.
      Set<Class<?>> safeTypes = safeTypes();

      Set<Class<?>> allCommandTypes = new HashSet<>();
      allCommandTypes.addAll(ControllerCoreCommandRandomTools.getInverseDynamicsCommandTypes(InverseDynamicsCommandBuffer.class));
      allCommandTypes.addAll(ControllerCoreCommandRandomTools.getInverseKinematicsCommandTypes(InverseKinematicsCommandBuffer.class));
      allCommandTypes.addAll(ControllerCoreCommandRandomTools.getVirtualModelControlCommandTypes(VirtualModelControlCommandBuffer.class,
                                                                                                 VirtualEffortCommand.class));
      allCommandTypes.addAll(ControllerCoreCommandRandomTools.getFeedbackControlCommandTypes(FeedbackControlCommandBuffer.class));

      String errorMessage = "";

      for (Class<?> typeToTest : collectTypesAndSubTypes(allCommandTypes, safeTypes))
      {
         if (typeToTest.isInterface() || safeTypes.contains(typeToTest))
         {
            if (verbose)
               LogTools.info("Skipping: " + typeToTest.getSimpleName());
            continue;
         }

         Object typeRandomInstanceA = ControllerCoreCommandRandomTools.nextTypeInstance(typeToTest, random, true, rootBody, referenceFrames);
         Object typeRandomInstanceB = ControllerCoreCommandRandomTools.nextTypeInstance(typeToTest, random, true, rootBody, referenceFrames);

         for (int attempt = 0; attempt < 5; attempt++)
         {
            if (!typeRandomInstanceA.equals(typeRandomInstanceB))
               break;
            typeRandomInstanceB = ControllerCoreCommandRandomTools.nextTypeInstance(typeToTest, random, true, rootBody, referenceFrames);
         }

         Method copySetter;
         try
         {
            copySetter = typeToTest.getDeclaredMethod("set", typeToTest);
         }
         catch (NoSuchMethodException e)
         {
            throw new AssertionFailedError("Could not find a copy setter method for the type: " + typeToTest.getSimpleName());
         }

         copySetter.invoke(typeRandomInstanceA, typeRandomInstanceB);
         if (!typeRandomInstanceA.equals(typeRandomInstanceB))
            errorMessage += "It seems that " + typeToTest.getSimpleName() + ".set(" + typeToTest.getSimpleName() + ") is missing to copy some of the fields.\n";
      }

      if (!errorMessage.isEmpty())
         fail("The following issues were detected:\n" + errorMessage);
   }

   public static void extractTypeSubTypes(Class<?> type, Map<Class<?>, Class<?>> subTypeToOwnerTypeMapToPack, Collection<Class<?>> typesToStopRecursionAt)
   {
      for (Field field : type.getDeclaredFields())
      {
         Class<?> fieldType = field.getType();

         if (fieldType.isPrimitive() || fieldType.isEnum())
            continue;
         if (subTypeToOwnerTypeMapToPack.containsKey(fieldType))
            continue;
         subTypeToOwnerTypeMapToPack.put(fieldType, type);

         if (typesToStopRecursionAt.contains(fieldType))
            continue;

         extractTypeSubTypes(fieldType, subTypeToOwnerTypeMapToPack, typesToStopRecursionAt);
      }
   }

   public static Collection<Class<?>> collectTypesAndSubTypes(Collection<Class<?>> types, Collection<Class<?>> typesToStopRecursionAt)
   {
      Collection<Class<?>> typesAndSubTypes = new HashSet<>();
      types.forEach(type -> collectTypeAndSubTypes(type, typesAndSubTypes, typesToStopRecursionAt));
      return typesAndSubTypes;
   }

   public static void collectTypeAndSubTypes(Class<?> type, Collection<Class<?>> typesAndSubTypesToPack, Collection<Class<?>> typesToStopRecursionAt)
   {
      if (type.isPrimitive() || type.isEnum())
         return;
      if (!typesAndSubTypesToPack.add(type))
         return;
      if (typesToStopRecursionAt.contains(type))
         return;

      for (Field field : type.getDeclaredFields())
      {
         collectTypeAndSubTypes(field.getType(), typesAndSubTypesToPack, typesToStopRecursionAt);
      }
   }

   private static Set<Class<?>> safeTypes()
   {
      Set<Class<?>> safeTypes = new HashSet<>();
      safeTypes.add(ArrayList.class);
      safeTypes.add(TDoubleArrayList.class);
      safeTypes.add(ReferenceFrame.class);
      safeTypes.add(FrameTupleArrayList.class);
      safeTypes.add(RecyclingArrayList.class);
      safeTypes.add(DenseMatrix64F.class);
      safeTypes.add(DenseMatrixArrayList.class);
      safeTypes.add(FramePoint3D.class);
      safeTypes.add(FrameVector3D.class);
      safeTypes.add(FramePoint2D.class);
      safeTypes.add(FrameVector2D.class);
      safeTypes.add(FrameQuaternion.class);
      safeTypes.add(Point3D.class);
      safeTypes.add(Vector3D.class);
      safeTypes.add(Point2D.class);
      safeTypes.add(Vector2D.class);
      safeTypes.add(FrameMatrix3D.class);
      safeTypes.add(FramePose3D.class);
      safeTypes.add(RigidBodyTransform.class);
      safeTypes.add(JointBasics.class);
      safeTypes.add(OneDoFJointBasics.class);
      safeTypes.add(RigidBodyBasics.class);
      safeTypes.add(Wrench.class);
      safeTypes.add(SpatialForce.class);
      safeTypes.add(double[].class);
      return safeTypes;
   }
}
