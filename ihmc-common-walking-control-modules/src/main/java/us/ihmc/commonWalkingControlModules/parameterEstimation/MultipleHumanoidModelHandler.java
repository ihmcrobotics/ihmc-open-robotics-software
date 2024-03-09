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

public class MultipleHumanoidModelHandler<T extends Enum<T>>
{
   private final Map<T, FullHumanoidRobotModel> modelMap;
   private final Map<T, RigidBodyBasics[]> bodyArrayMap;
   private final Map<T, List<? extends JointBasics>> jointListMap;

   MultipleHumanoidModelHandler(Class<T> enumClass)
   {
      modelMap = new EnumMap<>(enumClass);
      bodyArrayMap = new EnumMap<>(enumClass);
      jointListMap = new EnumMap<>(enumClass);
   }

   public void addRobotModel(T key, FullHumanoidRobotModel model)
   {
      modelMap.put(key, model);
      bodyArrayMap.put(key, model.getRootBody().subtreeArray());
      jointListMap.put(key, model.getRootJoint().subtreeList());
   }

   public void copyJointsState(T source, T destination, JointStateType type)
   {
      MultiBodySystemTools.copyJointsState(jointListMap.get(source), jointListMap.get(destination), type);
   }

   public FullHumanoidRobotModel getRobotModel(T key)
   {
      return modelMap.get(key);
   }

   public RigidBodyBasics[] getBodyArray(T key)
   {
      return bodyArrayMap.get(key);
   }

   public List<? extends JointBasics> getJointList(T key)
   {
      return jointListMap.get(key);
   }

   public void extractJointsState(T key, JointStateType type, DMatrix matrixToPack)
   {
      MultiBodySystemTools.extractJointsState(jointListMap.get(key), type, matrixToPack);
   }

   public void insertJointsState(T key, JointStateType type, DMatrix matrix)
   {
      MultiBodySystemTools.insertJointsState(jointListMap.get(key), type, matrix);
   }
}
