package us.ihmc.commonWalkingControlModules.controllerCore.command;

import static org.junit.jupiter.api.Assertions.fail;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Random;
import java.util.Set;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.ejml.data.DenseMatrix64F;
import org.junit.jupiter.api.Test;
import org.opentest4j.AssertionFailedError;

import com.google.common.base.CaseFormat;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
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
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.lists.DenseMatrixArrayList;
import us.ihmc.robotics.lists.FrameTupleArrayList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ControllerCoreCommandCodeQualityTest
{
   /**
    * Number of attempts used when comparing 2 random objects before failing.
    * <p>
    * Crank this up when in case this test is flaky.
    * </p>
    */
   private static final int NUMBER_OF_ATTEMPTS = 50;

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

      for (Class<?> commandType : ControllerCoreCommandRandomTools.getAdditionalClassesToTest())
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

   /**
    * This first test for the equals method asserts that:
    * <ul>
    * <li>1 instance of a command is equal to itself.
    * <li>2 instances of the same command obtained with the empty constructor are equal.
    * </ul>
    * 
    * @throws Exception
    */
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

      for (Class<?> commandType : ControllerCoreCommandRandomTools.getAdditionalClassesToTest())
      {
         errorMessage += testEqualsWithEmptyObject(commandType);
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

   /**
    * This second test for the equals method works as follows:
    * <ol>
    * <li>2 random instances of the same of command type are generated such that we know they are
    * equal.
    * <li>A field of one of the 2 commands is randomly modified.
    * <li>Given a number of attempts, if the two commands are equal, i.e.
    * {@code commandA.equals(commandB) == true}, then the equals method is assumed to not properly
    * check the field being modified.
    * <li>We loop over all controller core commands, and over all of each command's field.
    * </ol>
    * 
    * @throws Exception
    */
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

      // Low-level types or types from 3rd party libraries assumed to be safe.
      Set<Class<?>> safeTypes = safeTypes();
      Set<Class<?>> allCommandTypes = ControllerCoreCommandRandomTools.getAllCommandTypesWithoutBuffersAndInterfaces();
      String errorMessage = "";

      for (Class<?> typeToTest : collectTypesAndSubTypes(allCommandTypes, safeTypes))
      {
         if (safeTypes.contains(typeToTest))
         {
            if (verbose)
               LogTools.info("Skipping: " + typeToTest.getSimpleName());
            continue;
         }

         for (Field field : getAllFields(typeToTest))
         {
            // A static field is considered as non-representative of the object state.
            if (Modifier.isStatic(field.getModifiers()) || Modifier.isTransient(field.getModifiers()))
            {
               if (verbose)
                  LogTools.info("Skipping: " + typeToTest.getSimpleName() + "." + field.getName());
               continue;
            }

            if (verbose)
               LogTools.info("Testing : " + typeToTest.getSimpleName() + "." + field.getName());

            boolean hasEqualsFailed = false;

            // We give a few tries assuming that sometimes a field's value may not actually change after being randomly regenerated.
            for (int attempt = 0; attempt < NUMBER_OF_ATTEMPTS; attempt++)
            {
               long seed = random.nextLong();
               // Generating 2 objects that we know are the same because we use a fresh random with the same seed both times.
               Object typeRandomInstance = ControllerCoreCommandRandomTools.nextTypeInstance(typeToTest, new Random(seed), true, rootBody, referenceFrames);
               Object typeRandomInstanceDuplicate = ControllerCoreCommandRandomTools.nextTypeInstance(typeToTest, new Random(seed), true, rootBody,
                                                                                                      referenceFrames);

               try
               { // We randomize the field's value on the first random object.
                  ControllerCoreCommandRandomTools.randomizeField(random, field, typeRandomInstance, rootBody, referenceFrames);
               }
               catch (UnsupportedOperationException e)
               {
                  throw new AssertionFailedError("Encountered issue when randomizing: " + typeToTest.getSimpleName() + "." + field.getName()
                        + ". Probably missing a random generator for the type: " + field.getType(), e);
               }

               if (!typeRandomInstance.equals(typeRandomInstanceDuplicate))
               { // The equals method detected that we changed the field's value => the field is covered.
                  hasEqualsFailed = true;
                  break;
               }
            }

            // The equals method never detected the changed, we make a report that'll be printed at the end of this test.
            if (!hasEqualsFailed)
               errorMessage += typeToTest.getSimpleName() + ".equals(Object) seems to be missing test for the field: " + field.getName() + "\n";
         }
      }

      if (!errorMessage.isEmpty())
         fail("The following issues were detected:\n" + errorMessage);
   }

   /**
    * API test to verify that {@link ControllerCoreCommandRandomTools} declares the random generators
    * we need to generate controller core commands, including their fields.
    * 
    * @throws Exception
    */
   @Test
   void testRandomGeneratorsAPI() throws Exception
   {
      Set<Class<?>> typesToIgnore = new HashSet<>();
      typesToIgnore.add(ReferenceFrame.class);
      typesToIgnore.add(JointBasics.class);
      typesToIgnore.add(OneDoFJointBasics.class);
      typesToIgnore.add(RigidBodyBasics.class);

      Set<Class<?>> allCommandTypes = ControllerCoreCommandRandomTools.getAllCommandTypesWithoutBuffersAndInterfaces();
      allCommandTypes.removeIf(type -> type.isInterface());
      Map<Class<?>, Class<?>> typesToVerify = new HashMap<>();
      for (Class<?> commandType : allCommandTypes)
         typesToVerify.put(commandType, null);
      for (Class<?> commandType : allCommandTypes)
      {
         extractTypeSubTypes(commandType, typesToVerify, safeTypes());
      }

      String errorMessage = "";

      Set<String> randomGeneratorNames = Stream.of(ControllerCoreCommandRandomTools.class.getDeclaredMethods()).map(Method::getName)
                                               .collect(Collectors.toSet());

      for (Entry<Class<?>, Class<?>> typeToVerifyAndOwnerEntry : typesToVerify.entrySet())
      {
         Class<?> typeToVerify = typeToVerifyAndOwnerEntry.getKey();
         Class<?> owner = typeToVerifyAndOwnerEntry.getValue();

         if (typesToIgnore.contains(typeToVerify))
            continue;

         String typeName = typeToVerify.getSimpleName();
         if (typeToVerify.isArray())
         {// This allows to properly retrieve the expected generator name for types such as "double[]".
            typeName = CaseFormat.LOWER_CAMEL.to(CaseFormat.UPPER_CAMEL, typeToVerify.getComponentType().getSimpleName()) + "Array";
         }
         List<String> expectedGeneratorNames = Arrays.asList("next" + typeName, "randomize" + typeName);

         if (!randomGeneratorNames.removeAll(expectedGeneratorNames))
         {
            errorMessage += "Missing random generator for: " + typeName;
            if (owner != null)
               errorMessage += " (discovered in type: " + owner.getSimpleName() + ")";
            errorMessage += "\n";
         }
      }

      if (!errorMessage.isEmpty())
         fail("The following issues were detected:\n" + errorMessage);
   }

   /**
    * This tests asserts that the random generators in {@link ControllerCoreCommandRandomTools}
    * generate fully random objects, i.e. each field of a random object is generated randomly.
    * <ol>
    * <li>2 objects of the same type are generated randomly.
    * <li>we then verify that each field is has different value in both objects.
    * <li>as previously, we give the random generator a few shots before declaring it as buggy.
    * </ol>
    * 
    * @throws Exception
    */
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

      // Low-level types or types from 3rd party libraries assumed to be safe.
      Set<Class<?>> safeTypes = safeTypes();
      Set<Class<?>> allCommandTypes = ControllerCoreCommandRandomTools.getAllCommandTypesWithoutBuffersAndInterfaces();
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

         for (Field field : getAllFields(typeToTest))
         {
            if (Modifier.isStatic(field.getModifiers()) || Modifier.isTransient(field.getModifiers()))
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
            for (int attempt = 0; attempt < NUMBER_OF_ATTEMPTS; attempt++)
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

            for (int attempt = 0; attempt < NUMBER_OF_ATTEMPTS; attempt++)
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

   /**
    * This test asserts that each command properly implement a copy setter, i.e. each field is copied
    * from one command to the other.
    * <p>
    * Since we now know that the equals method of the commands is good and that our random generators
    * can generate fully random objects, all we have to do is:
    * <ul>
    * <li>Generate 2 random objects of the same type.
    * <li>Invoke the copy setter on one of them given the other object.
    * <li>Assert the two objects are equal.
    * <li>Try a few times for each type to reduce the chance of generating field's with the same value
    * twice.
    * </ul>
    * </p>
    * 
    * @throws Exception
    */
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

      // Low-level types or types from 3rd party libraries assumed to be safe.
      Set<Class<?>> safeTypes = safeTypes();
      Set<Class<?>> allCommandTypes = ControllerCoreCommandRandomTools.getAllCommandTypesWithoutBuffersAndInterfaces();
      String errorMessage = "";

      for (Class<?> typeToTest : collectTypesAndSubTypes(allCommandTypes, safeTypes))
      {
         if (typeToTest.isInterface() || safeTypes.contains(typeToTest))
         {
            if (verbose)
               LogTools.info("Skipping: " + typeToTest.getSimpleName());
            continue;
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

         Object typeRandomInstanceA = ControllerCoreCommandRandomTools.nextTypeInstance(typeToTest, random, true, rootBody, referenceFrames);

         for (int attempt = 0; attempt < NUMBER_OF_ATTEMPTS; attempt++)
         {
            Object typeRandomInstanceB = ControllerCoreCommandRandomTools.nextTypeInstance(typeToTest, random, true, rootBody, referenceFrames);

            copySetter.invoke(typeRandomInstanceA, typeRandomInstanceB);

            if (!typeRandomInstanceA.equals(typeRandomInstanceB))
            {
               errorMessage += "It seems that " + typeToTest.getSimpleName() + ".set(" + typeToTest.getSimpleName()
                     + ") is missing to copy some of the fields.\n";
               break;
            }
         }
      }

      if (!errorMessage.isEmpty())
         fail("The following issues were detected:\n" + errorMessage);
   }

   public static void extractTypeSubTypes(Class<?> type, Map<Class<?>, Class<?>> subTypeToOwnerTypeMapToPack, Collection<Class<?>> typesToStopRecursionAt)
         throws Exception
   {
      for (Field field : getAllFields(type))
      {
         Class<?> fieldType = field.getType();

         if (fieldType.isPrimitive() || fieldType.isEnum())
            continue;
         if (Modifier.isStatic(field.getModifiers()))
            continue;
         if (Modifier.isTransient(field.getModifiers()))
            continue;
         if (subTypeToOwnerTypeMapToPack.containsKey(fieldType))
            continue;
         subTypeToOwnerTypeMapToPack.put(fieldType, type);

         if (typesToStopRecursionAt.contains(fieldType))
         {
            Class<?> extractedGenericType = extractGenericType(type, field);
            if (extractedGenericType != null)
            {
               subTypeToOwnerTypeMapToPack.put(fieldType, type);
               if (!typesToStopRecursionAt.contains(extractedGenericType))
               {
                  extractTypeSubTypes(extractedGenericType, subTypeToOwnerTypeMapToPack, typesToStopRecursionAt);
               }
            }
            continue;
         }

         extractTypeSubTypes(fieldType, subTypeToOwnerTypeMapToPack, typesToStopRecursionAt);
      }
   }

   public static Collection<Class<?>> collectTypesAndSubTypes(Collection<Class<?>> types, Collection<Class<?>> typesToStopRecursionAt) throws Exception
   {
      Collection<Class<?>> typesAndSubTypes = new HashSet<>();
      for (Class<?> type : types)
      {
         collectTypeAndSubTypes(type, typesAndSubTypes, typesToStopRecursionAt);
      }
      return typesAndSubTypes;
   }

   public static void collectTypeAndSubTypes(Class<?> type, Collection<Class<?>> typesAndSubTypesToPack, Collection<Class<?>> typesToStopRecursionAt)
         throws Exception
   {
      if (type.isPrimitive() || type.isEnum())
         return;
      if (!typesAndSubTypesToPack.add(type))
         return;
      if (typesToStopRecursionAt.contains(type))
         return;

      for (Field field : getAllFields(type))
      {
         if (Modifier.isStatic(field.getModifiers()))
            continue;
         if (Modifier.isTransient(field.getModifiers()))
            continue;

         Class<?> extractedGenericType = extractGenericType(type, field);
         if (extractedGenericType != null)
            collectTypeAndSubTypes(extractedGenericType, typesAndSubTypesToPack, typesToStopRecursionAt);

         collectTypeAndSubTypes(field.getType(), typesAndSubTypesToPack, typesToStopRecursionAt);
      }
   }

   private static Set<Class<?>> safeTypes()
   {
      Set<Class<?>> safeTypes = new HashSet<>();
      safeTypes.add(ArrayList.class);
      safeTypes.add(TDoubleArrayList.class);
      safeTypes.add(ReferenceFrame.class);
      safeTypes.add(MovingReferenceFrame.class);
      safeTypes.add(FrameTupleArrayList.class);
      safeTypes.add(RecyclingArrayList.class);
      safeTypes.add(SideDependentList.class);
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
      safeTypes.add(PrismaticJoint.class);
      safeTypes.add(RevoluteJoint.class);
      safeTypes.add(RigidBodyBasics.class);
      safeTypes.add(Wrench.class);
      safeTypes.add(SpatialForce.class);
      safeTypes.add(ConvexPolygon2D.class);
      safeTypes.add(double[].class);
      return safeTypes;
   }

   private static List<Field> getAllFields(Class<?> clazz)
   {
      if (clazz == null)
         return Collections.emptyList();
      List<Field> declaredFields = new ArrayList<Field>(Arrays.asList(clazz.getDeclaredFields()));
      declaredFields.addAll(getAllFields(clazz.getSuperclass()));
      return declaredFields;
   }

   public static Class<?> extractGenericType(Class<?> ownerType, Field field) throws Exception
   {
      Class<?> fieldType = field.getType();
      if (fieldType == RecyclingArrayList.class)
      {
         field.setAccessible(true);
         Object owner = ownerType.newInstance();
         Object fieldInstance = field.get(owner);
         Field allocatorField = RecyclingArrayList.class.getDeclaredField("allocator");
         allocatorField.setAccessible(true);
         return ((Supplier<?>) allocatorField.get(fieldInstance)).get().getClass();
      }
      if (fieldType == List.class)
      {
         field.setAccessible(true);
         Random random = new Random();
         List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, 2);
         Object ownerInstance = ControllerCoreCommandRandomTools.nextTypeInstance(ownerType, random, true,
                                                                                  joints.get(0).getSuccessor(),
                                                                                  ReferenceFrame.getWorldFrame());
         Object fieldInstance = field.get(ownerInstance);
         Object object = ((List<?>) fieldInstance).get(0);
         if (object == null)
            fail("Random generator for " + ownerType.getSimpleName() + " did not instantiate fields in list " + field.getName() + ".");
         return object.getClass();
      }
      if (fieldType == SideDependentList.class)
      {
         field.setAccessible(true);
         Object ownerInstance = ControllerCoreCommandRandomTools.nextTypeInstance(ownerType, new Random(), true,
                                                                                  new RigidBody("Dummy", ReferenceFrame.getWorldFrame()),
                                                                                  ReferenceFrame.getWorldFrame());
         Object fieldInstance = field.get(ownerInstance);
         Object object = ((SideDependentList<?>) fieldInstance).get(RobotSide.LEFT);
         if (object == null)
            fail("Random generator for " + ownerType.getSimpleName() + " did not instantiate fields in side dependent list " + field.getName() + ".");
         return object.getClass();
      }

      // TODO: Add a check that all safe types with generics extract their generics here.

      return null;
   }
}
