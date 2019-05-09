package us.ihmc.humanoidBehaviors.upDownExploration;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;

import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class UpDownResult
{
   private List<Pose3D> polygonPoints3D;
   private boolean valid = false;

   public UpDownResult()
   {
      // for kryo
   }

   public UpDownResult(int numberOfVertices)
   {
      polygonPoints3D = Stream.generate(Pose3D::new).limit(numberOfVertices + 1).collect(Collectors.toList());
   }

   public void reset()
   {
      valid = false;
      polygonPoints3D.forEach(Pose3DBasics::setToZero);
   }

   public void setValid(boolean valid)
   {
      this.valid = valid;
   }

   public boolean isValid()
   {
      return valid;
   }

   public List<Pose3D> getPoints()
   {
      return polygonPoints3D;
   }
}
