package us.ihmc.robotDataLogger.jointState;

import java.nio.DoubleBuffer;
import java.nio.LongBuffer;
import java.util.List;

import us.ihmc.robotDataLogger.JointType;


public abstract class JointState
{
   
   private final JointType type;
   private final String name;

   public JointState(String name, JointType type)
   {
      this.name = name;
      this.type  = type;
   }

   public String getName()
   {
      return name;
   }

   public JointType getType()
   {
      return type;
   }

   public abstract void update(LongBuffer buffer);
   
   public abstract void update(DoubleBuffer buffer);

   public abstract void get(double[] array);
   
   public abstract void get(LongBuffer buffer);
   
   public abstract int getNumberOfStateVariables();

   public static int getNumberOfVariables(JointType type)
   {
      switch (type)
      {
      case OneDoFJoint:
         return OneDoFState.numberOfStateVariables;
      case SiXDoFJoint:
         return SixDoFState.numberOfStateVariables;
      default:
         throw new RuntimeException("Unknown joint type" + type);
      }
   }
      
   public static JointState createJointState(String name, JointType type)
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
   
   public static int getNumberOfJointStates(List<JointState> jointStates)
   {
      int numberOfJointStates = 0;
      for(int i = 0; i < jointStates.size(); i++)
      {
        numberOfJointStates += jointStates.get(i).getNumberOfStateVariables();
      }
      return numberOfJointStates;
   }
}
