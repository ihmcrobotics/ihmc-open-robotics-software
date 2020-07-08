package us.ihmc.robotics.physics;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;

public interface MultiBodySystemStateWriter
{
   void setMultiBodySystem(MultiBodySystemBasics multiBodySystem);

   void write();

   public static <T extends JointBasics> MultiBodySystemStateWriter singleJointStateWriter(String jointName, Consumer<T> jointStateWriter)
   {
      return new MultiBodySystemStateWriter()
      {
         private T joint;

         @Override
         public void write()
         {
            jointStateWriter.accept(joint);
         }

         @SuppressWarnings("unchecked")
         @Override
         public void setMultiBodySystem(MultiBodySystemBasics multiBodySystem)
         {
            joint = (T) RobotCollisionModel.findJoint(jointName, multiBodySystem);
         }
      };
   }

   public static abstract class MapBasedJointStateWriter implements MultiBodySystemStateWriter
   {
      private final Map<String, JointBasics> jointMap = new HashMap<>();

      @SuppressWarnings("unchecked")
      public <T extends JointBasics> T getJoint(String jointName)
      {
         return (T) jointMap.get(jointName);
      }

      @Override
      public void setMultiBodySystem(MultiBodySystemBasics multiBodySystem)
      {
         for (JointBasics joint : multiBodySystem.getAllJoints())
         {
            jointMap.put(joint.getName(), joint);
         }
      }
   }
}
