package us.ihmc.perception.detections.supervisePose;

import java.util.Objects;

public record SupervisePoseTarget(String category, String instance, String yoloClass)
{
   public SupervisePoseTarget
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