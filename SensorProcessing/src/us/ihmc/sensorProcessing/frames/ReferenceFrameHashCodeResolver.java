package us.ihmc.sensorProcessing.frames;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Collection;

import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSextant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class ReferenceFrameHashCodeResolver
{
   private final TLongObjectHashMap<ReferenceFrame> nameBasedHashCodeToReferenceFrameMap = new TLongObjectHashMap<ReferenceFrame>();

   public ReferenceFrameHashCodeResolver(FullRobotModel fullRobotModel, ReferenceFrames referenceFrames)
   {
      nameBasedHashCodeToReferenceFrameMap.put(ReferenceFrame.getWorldFrame().getNameBasedHashCode(), ReferenceFrame.getWorldFrame());

      Class<?> referenceFrameClass = referenceFrames.getClass();
      Class<?> fullRobotModelClass = fullRobotModel.getClass();
      try
      {
         referenceFrameExtractor(referenceFrames, referenceFrameClass);
         referenceFrameExtractor(fullRobotModel, fullRobotModelClass);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }

      // a little repetitive, but better than recursion
      for (OneDoFJoint joint : fullRobotModel.getOneDoFJoints())
      {
         ReferenceFrame frameBeforeJoint = joint.getFrameBeforeJoint();
         ReferenceFrame frameAfterJoint = joint.getFrameAfterJoint();
         ReferenceFrame comLinkBefore = joint.getPredecessor().getBodyFixedFrame();
         ReferenceFrame comLinkAfter = joint.getSuccessor().getBodyFixedFrame();

         nameBasedHashCodeToReferenceFrameMap.put(frameBeforeJoint.getNameBasedHashCode(), frameBeforeJoint);
         nameBasedHashCodeToReferenceFrameMap.put(frameAfterJoint.getNameBasedHashCode(), frameAfterJoint);
         nameBasedHashCodeToReferenceFrameMap.put(comLinkBefore.getNameBasedHashCode(), comLinkBefore);
         nameBasedHashCodeToReferenceFrameMap.put(comLinkAfter.getNameBasedHashCode(), comLinkAfter);
      }
   }

   private void referenceFrameExtractor(Object obj, Class<?> clazz) throws IllegalAccessException, InvocationTargetException
   {
      Method[] declaredMethods = clazz.getMethods();
      for (Method method : declaredMethods)
      {
         if (method.getReturnType() == ReferenceFrame.class)
         {
            if (method.getParameterCount() == 0)
            {
               ReferenceFrame referenceFrame = (ReferenceFrame) method.invoke(obj);
               nameBasedHashCodeToReferenceFrameMap.put(referenceFrame.getNameBasedHashCode(), referenceFrame);
            }
            else if (method.getParameterCount() == 1 && method.getParameterTypes()[0] == RobotSide.class)
            {
               for (RobotSide robotSide : RobotSide.values)
               {
                  ReferenceFrame referenceFrame = (ReferenceFrame) method.invoke(obj, robotSide);
                  nameBasedHashCodeToReferenceFrameMap.put(referenceFrame.getNameBasedHashCode(), referenceFrame);
               }
            }
            else if (method.getParameterCount() == 1 && method.getParameterTypes()[0] == RobotQuadrant.class)
            {
               for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
               {
                  ReferenceFrame referenceFrame = (ReferenceFrame) method.invoke(obj, robotQuadrant);
                  nameBasedHashCodeToReferenceFrameMap.put(referenceFrame.getNameBasedHashCode(), referenceFrame);
               }
            }
            else if (method.getParameterCount() == 1 && method.getParameterTypes()[0] == RobotSextant.class)
            {
               for (RobotSextant robotSextant : RobotSextant.values)
               {
                  ReferenceFrame referenceFrame = (ReferenceFrame) method.invoke(obj, robotSextant);
                  nameBasedHashCodeToReferenceFrameMap.put(referenceFrame.getNameBasedHashCode(), referenceFrame);
               }
            }
         }
      }
   }

   public ReferenceFrame getReferenceFrameFromNameBaseHashCode(long nameBasedHashCode)
   {
      return nameBasedHashCodeToReferenceFrameMap.get(nameBasedHashCode);
   }

   public Collection<ReferenceFrame> getAllReferenceFrames()
   {
      return nameBasedHashCodeToReferenceFrameMap.valueCollection();
   }
}
