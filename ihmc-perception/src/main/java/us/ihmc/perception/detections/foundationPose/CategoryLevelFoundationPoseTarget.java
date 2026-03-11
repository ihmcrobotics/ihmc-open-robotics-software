package us.ihmc.perception.detections.foundationPose;

import java.util.Objects;

public record CategoryLevelFoundationPoseTarget(String category, String instance, String yoloClass)
{
   public CategoryLevelFoundationPoseTarget
   {
      Objects.requireNonNull(category);
      Objects.requireNonNull(instance);
      Objects.requireNonNull(yoloClass);
   }

   public String key()
   {
      return category + "/" + instance;
   }

   public String title()
   {
      return category + ":" + instance;
   }
}