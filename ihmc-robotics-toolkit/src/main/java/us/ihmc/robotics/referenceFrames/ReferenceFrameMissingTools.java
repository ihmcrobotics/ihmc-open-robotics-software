package us.ihmc.robotics.referenceFrames;

import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

import java.util.HashMap;

public class ReferenceFrameMissingTools
{
   private static final ThreadLocal<HashMap<String, MutableInt>> INDEX = ThreadLocal.withInitial(HashMap::new);

   public static ReferenceFrame constructFrameWithUnchangingTransformFromParent(ReferenceFrame parentFrame, RigidBodyTransformReadOnly transformFromParent)
   {
      return ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(computeFrameName(getCallingClassName()),
                                                                                 parentFrame,
                                                                                 transformFromParent);
   }

   public static ReferenceFrame constructFrameWithUnchangingTransformToParent(ReferenceFrame parentFrame, RigidBodyTransformReadOnly transformToParent)
   {
      return ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(computeFrameName(getCallingClassName()),
                                                                               parentFrame,
                                                                               transformToParent);
   }

   public static ReferenceFrame constructFrameWithChangingTransformFromParent(ReferenceFrame parentFrame, RigidBodyTransformReadOnly transformFromParent)
   {
      return ReferenceFrameTools.constructFrameWithChangingTransformFromParent(computeFrameName(getCallingClassName()),
                                                                               parentFrame,
                                                                               transformFromParent);
   }

   public static ReferenceFrame constructFrameWithChangingTransformToParent(ReferenceFrame parentFrame, RigidBodyTransformReadOnly transformToParent)
   {
      return ReferenceFrameTools.constructFrameWithChangingTransformToParent(computeFrameName(getCallingClassName()),
                                                                             parentFrame,
                                                                             transformToParent);
   }

   public static ReferenceFrame constructFrameWithChangingTransformToParent(ReferenceFrame parentFrame,
                                                                            RigidBodyTransformReadOnly transformToParent,
                                                                            String callingClassName)
   {
      return ReferenceFrameTools.constructFrameWithChangingTransformToParent(computeFrameName(callingClassName), parentFrame, transformToParent);
   }

   public static ReferenceFrame constructARootFrame()
   {
      return ReferenceFrameTools.constructARootFrame(computeFrameName(getCallingClassName()));
   }

   /**
    * To be used externally. Remember {@link #getCallingClassName()}
    * depends on the stacktrace, so don't try and call this from the
    * constructor to reduce code duplication.
    */
   public static String computeFrameName()
   {
      return computeFrameName(getCallingClassName());
   }

   private static String computeFrameName(String callingClassName)
   {
      HashMap<String, MutableInt> threadLocalHashMap = INDEX.get();
      threadLocalHashMap.computeIfAbsent(callingClassName, key -> new MutableInt());
      int frameID = threadLocalHashMap.get(callingClassName).getAndIncrement();
      return callingClassName + frameID;
   }

   /**
    * Gets the calling class name by creating a Throwable.
    */
   private static String getCallingClassName()
   {
      StackTraceElement callerStackElement = new Throwable().getStackTrace()[2];
      return callerStackElement.getClassName();
   }
}
