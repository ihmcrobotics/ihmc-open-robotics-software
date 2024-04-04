package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrix;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;

import java.util.EnumMap;
import java.util.List;
import java.util.Map;

/**
 * This class is used to handle multiple {@link FullHumanoidRobotModel}s and easily copy state between them.
 *
 * @param <T> the type of the enum used to identify the purpose of the different robot models.
 *
 * @author James Foster
 */
public class MultipleHumanoidModelHandler<T extends Enum<T>>
{
   /** The map from the enum to the robot model. */
   private final Map<T, FullHumanoidRobotModel> modelMap;
   /** The map from the enum to the array of rigid bodies in each robot model. */
   private final Map<T, RigidBodyBasics[]> bodyArrayMap;
   /** The map from the enum to the list of joints in each robot model. */
   private final Map<T, List<? extends JointBasics>> jointListMap;

   public MultipleHumanoidModelHandler(Class<T> enumClass)
   {
      modelMap = new EnumMap<>(enumClass);
      bodyArrayMap = new EnumMap<>(enumClass);
      jointListMap = new EnumMap<>(enumClass);
   }

   public void putRobotModel(T key, FullHumanoidRobotModel model)
   {
      modelMap.put(key, model);
      bodyArrayMap.put(key, model.getRootBody().subtreeArray());
      jointListMap.put(key, model.getRootJoint().subtreeList());
   }

   public void copyJointsState(T source, T destination, JointStateType type)
   {
      MultiBodySystemTools.copyJointsState(jointListMap.get(source), jointListMap.get(destination), type);
   }

   public void extractJointsState(T key, JointStateType type, DMatrix matrixToPack)
   {
      MultiBodySystemTools.extractJointsState(jointListMap.get(key), type, matrixToPack);
   }

   public void insertJointsState(T key, JointStateType type, DMatrix matrix)
   {
      MultiBodySystemTools.insertJointsState(jointListMap.get(key), type, matrix);
   }

   public FullHumanoidRobotModel getRobotModel(T key)
   {
      return modelMap.get(key);
   }

   public RigidBodyBasics[] getBodyArray(T key)
   {
      return bodyArrayMap.get(key);
   }
}
