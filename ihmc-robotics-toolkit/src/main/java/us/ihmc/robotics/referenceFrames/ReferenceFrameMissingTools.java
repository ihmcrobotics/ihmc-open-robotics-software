package us.ihmc.robotics.referenceFrames;

import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;

import java.util.HashMap;

public class ReferenceFrameMissingTools
{
   private static final ThreadLocal<HashMap<String, MutableInt>> INDEX = ThreadLocal.withInitial(HashMap::new);

   public static ReferenceFrame constructFrameWithUnchangingTransformFromParent(ReferenceFrame parentFrame, RigidBodyTransform transformFromParent)
   {
      return ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(computeFrameName(getCallingClassName()),
                                                                                 parentFrame,
                                                                                 transformFromParent);
   }

   public static ReferenceFrame constructFrameWithUnchangingTransformToParent(ReferenceFrame parentFrame, RigidBodyTransform transformToParent)
   {
      return ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(computeFrameName(getCallingClassName()),
                                                                               parentFrame,
                                                                               transformToParent);
   }

   public static ReferenceFrame constructFrameWithChangingTransformFromParent(ReferenceFrame parentFrame, RigidBodyTransform transformFromParent)
   {
      return ReferenceFrameTools.constructFrameWithChangingTransformFromParent(computeFrameName(getCallingClassName()),
                                                                               parentFrame,
                                                                               transformFromParent);
   }

   public static ReferenceFrame constructFrameWithChangingTransformToParent(ReferenceFrame parentFrame, RigidBodyTransform transformToParent)
   {
      return ReferenceFrameTools.constructFrameWithChangingTransformToParent(computeFrameName(getCallingClassName()),
                                                                             parentFrame,
                                                                             transformToParent);
   }

   private static String computeFrameName(String callingClassName)
   {
      HashMap<String, MutableInt> threadLocalHashMap = INDEX.get();
      threadLocalHashMap.computeIfAbsent(callingClassName, key -> new MutableInt());
      int frameID = threadLocalHashMap.get(callingClassName).getAndIncrement();
      return callingClassName + frameID;
   }

   private static String getCallingClassName()
   {
      StackTraceElement callerStackElement = new Throwable().getStackTrace()[2];
      return callerStackElement.getClassName();
   }
}
