package us.ihmc.robotEnvironmentAwareness.slam;

import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;

import java.util.ArrayList;
import java.util.List;

public class SLAMFrameState
{
   private Point3DReadOnly[] uncorrectedPointCloudInWorld = new Point3DReadOnly[0];
   private List<? extends Point3DReadOnly> correctedPointCloudInWorld = new ArrayList<>();
   private Point3DReadOnly[] correspondingPointsInWorld = new Point3DReadOnly[0];

   private final Point3D sensorPosition = new Point3D();
   private final Orientation3DBasics sensorOrientation = new Quaternion();

   public SLAMFrameState()
   {
   }

   public Point3DBasics getSensorPosition()
   {
      return sensorPosition;
   }

   public Orientation3DBasics getSensorOrientation()
   {
      return sensorOrientation;
   }

   public void setUncorrectedPointCloudInWorld(Point3DReadOnly[] uncorrectedPointCloudInWorld)
   {
      this.uncorrectedPointCloudInWorld = uncorrectedPointCloudInWorld;
   }

   public void setCorrectedPointCloudInWorld(List<? extends Point3DReadOnly> correctedPointCloudInWorld)
   {
      this.correctedPointCloudInWorld = correctedPointCloudInWorld;
   }

   public void setCorrespondingPointsInWorld(Point3DReadOnly[] correspondingPointsInWorld)
   {
      this.correspondingPointsInWorld = correspondingPointsInWorld;
   }

   public Point3DReadOnly[] getUncorrectedPointCloudInWorld()
   {
      return uncorrectedPointCloudInWorld;
   }

   public List<? extends Point3DReadOnly> getCorrectedPointCloudInWorld()
   {
      return correctedPointCloudInWorld;
   }

   public Point3DReadOnly[] setCorrespondingPointsInWorld()
   {
      return correspondingPointsInWorld;
   }
}
