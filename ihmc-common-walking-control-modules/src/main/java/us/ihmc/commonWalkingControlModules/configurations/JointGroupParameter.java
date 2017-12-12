package us.ihmc.commonWalkingControlModules.configurations;

import java.util.List;

/**
 * This class is a data structure that can hold a parameter for a group of joints.
 * <p>
 * In case a single set of tunable parameters is used for a number of joints (the joint group)
 * the controller will only create YoVariables or Parameters once and use them for all joints
 * in the group.
 * </p>
 * <p>
 * For example, if all arm joints have a common parameter the name-prefix of the tunable
 * controller parameter will be the jointGroupName ('arm'). The joint name list should contain
 * all arm joints of the robot. The parameter {@code T} will contain the initial (for YoVariables)
 * or the default (for Parameters) values.
 * </p>
 * @param <T> is the type of parameter.
 */
public class JointGroupParameter<T>
{
   private final T parameter;
   private final String jointGroupName;
   private final List<String> jointNames;

   public JointGroupParameter(String jointGroupName, T parameter, List<String> jointNames)
   {
      this.jointGroupName = jointGroupName;
      this.parameter = parameter;
      this.jointNames = jointNames;
   }

   /**
    * Get the default value for the shared parameter.
    */
   public T getParameter()
   {
      return parameter;
   }

   /**
    * Get the name of the joint group that should share a single parameter.
    */
   public String getJointGroupName()
   {
      return jointGroupName;
   }

   /**
    * Get the names of the joints that share the parameter.
    */
   public List<String> getJointNames()
   {
      return jointNames;
   }
}
