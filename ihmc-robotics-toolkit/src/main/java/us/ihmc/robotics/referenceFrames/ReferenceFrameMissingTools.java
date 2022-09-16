package us.ihmc.robotics.referenceFrames;

import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class ReferenceFrameMissingTools
{
   private static final ThreadLocal<MutableInt> INDEX = ThreadLocal.withInitial(MutableInt::new);

   public static String getCallingClassName()
   {
      StackTraceElement callerStackElement = new Throwable().getStackTrace()[2];
      return callerStackElement.getClassName();
   }

   public static ReferenceFrame constructFrameWithUnchangingTransformFromParent(ReferenceFrame parentFrame, RigidBodyTransform transformFromParent)
   {
      return ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(getCallingClassName() + INDEX.get().getAndIncrement(),
                                                                                 parentFrame,
                                                                                 transformFromParent);
   }

   public static ReferenceFrame constructFrameWithUnchangingTransformToParent(ReferenceFrame parentFrame, RigidBodyTransform transformToParent)
   {
      return ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(getCallingClassName() + INDEX.get().getAndIncrement(),
                                                                               parentFrame,
                                                                               transformToParent);
   }

   public static ReferenceFrame constructFrameWithChangingTransformFromParent(ReferenceFrame parentFrame, RigidBodyTransform transformFromParent)
   {
      return ReferenceFrameTools.constructFrameWithChangingTransformFromParent(getCallingClassName() + INDEX.get().getAndIncrement(),
                                                                               parentFrame,
                                                                               transformFromParent);
   }

   public static ReferenceFrame constructFrameWithChangingTransformToParent(ReferenceFrame parentFrame, RigidBodyTransform transformToParent)
   {
      return ReferenceFrameTools.constructFrameWithChangingTransformToParent(getCallingClassName() + INDEX.get().getAndIncrement(),
                                                                             parentFrame,
                                                                             transformToParent);
   }
}
