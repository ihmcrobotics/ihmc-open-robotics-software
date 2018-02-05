package us.ihmc.commonWalkingControlModules.configurations;

import java.util.Collections;
import java.util.List;

/**
 * This class is a data structure that can hold a parameter that is shared by a group. This can
 * be a joint group or a body group for example. When tunable parameters are created, only one
 * tunable (through YoVariables or Parameters) implementation of the parameter {@code T} will
 * be instantiated and shared between all members of the group.
 * <p>
 * For example, if all arm joints have a common parameter then the name-prefix of the tunable
 * controller parameter will be the groupName ('arm'). The member name list should contain
 * all arm joint names of the robot. The parameter {@code T} will contain the initial (for
 * YoVariables) or the default (for Parameters) values.
 * </p>
 * @param <T> is the type of parameter.
 */
public class GroupParameter<T>
{
   private final T parameter;
   private final String groupName;
   private final List<String> memberNames;

   public GroupParameter(String groupAndMemberName)
   {
      this(groupAndMemberName, groupAndMemberName);
   }

   public GroupParameter(String groupName, String memberName)
   {
      this(groupName, Collections.singletonList(memberName));
   }

   public GroupParameter(String groupName, List<String> memberNames)
   {
      this(groupName, null, memberNames);
   }

   public GroupParameter(String groupName, T parameter, String memberName)
   {
      this(groupName, parameter, Collections.singletonList(memberName));
   }

   public GroupParameter(String groupName, T parameter, List<String> memberNames)
   {
      this.groupName = groupName;
      this.parameter = parameter;
      this.memberNames = memberNames;
   }

   /**
    * Whether this {@link #GroupParameter} was created with a parameter.
    */
   public boolean hasParameter()
   {
      return parameter != null;
   }

   /**
    * Get the default value for the shared parameter.
    */
   public T getParameter()
   {
      return parameter;
   }

   /**
    * Get the name of the group that should share a single parameter.
    */
   public String getGroupName()
   {
      return groupName;
   }

   /**
    * Get the names of the members of the group that share the parameter.
    */
   public List<String> getMemberNames()
   {
      return memberNames;
   }
}
