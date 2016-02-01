package us.ihmc.robotDataCommunication.jointState;

import java.nio.LongBuffer;
import java.util.List;

import us.ihmc.robotDataCommunication.generated.YoProtoHandshakeProto.YoProtoHandshake.JointDefinition.JointType;
import us.ihmc.simulationconstructionset.Joint;

public abstract class JointState<U extends Joint>
{
   private final String name;

   public JointState(String name)
   {
      this.name = name;
   }

   public String getName()
   {
      return name;
   }

   public abstract void update(LongBuffer buffer);

   public abstract void get(U joint);

   public abstract void get(double[] array);
   
   public abstract int getNumberOfStateVariables();

   public static int getNumberOfVariables(JointType type)
   {
      return createJointState(null, type).getNumberOfStateVariables();
   }
   
   public static JointState<?> createJointState(String name, JointType type)
   {
      switch (type)
      {
      case OneDoFJoint:
         return new OneDoFState(name);
      case SiXDoFJoint:
         return new SixDoFState(name);
      default:
         throw new RuntimeException("Unknown joint type" + type);
      }
   }
   
   public static int getNumberOfJointStates(List<JointState<? extends Joint>> jointStates)
   {
      int numberOfJointStates = 0;
      for(int i = 0; i < jointStates.size(); i++)
      {
        numberOfJointStates += jointStates.get(i).getNumberOfStateVariables();
      }
      return numberOfJointStates;
   }
}
