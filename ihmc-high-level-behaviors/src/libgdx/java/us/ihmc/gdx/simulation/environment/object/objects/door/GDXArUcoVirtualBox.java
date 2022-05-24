package us.ihmc.gdx.simulation.environment.object.objects.door;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.perception.OpenCVArUcoMarker;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

public class GDXArUcoVirtualBox extends GDXVirtualGhostObject
{
   private final OpenCVArUcoMarker arUcoMarker;
   private final RigidBodyTransform markerToWorld = new RigidBodyTransform();
   private final ReferenceFrame markerFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                                     markerToWorld);
   private final RigidBodyTransform transformToMarker = new RigidBodyTransform();
   private ReferenceFrame virtualFrame;

   public GDXArUcoVirtualBox(int id)
   {
      super("environmentObjects/box/box.g3dj");

      arUcoMarker = new OpenCVArUcoMarker(id, 0.210101);

      double x = 0.0;
      double y = -0.102365;
      double z = -0.062277;
      transformToMarker.set(new YawPitchRoll(Math.toRadians(180.0), 0.0, Math.toRadians(180.0)), new Point3D(x, y, z));

      virtualFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(markerFrame, transformToMarker);
   }

   @Override
   public void update()
   {
//      double x = 0.0;
//      double y = -0.102365 - 0.5;
//      double z = -0.062277;
      double x = 0.0 + 0.07;
      double y = 0.0 + 0.15;
      double z = 0.0 + 0.17;
      transformToMarker.set(new YawPitchRoll(Math.toRadians(180.0), Math.toRadians(0.0), Math.toRadians(-90.0)), new Point3D(x, y, z));
      virtualFrame.update();

      markerFrame.update();

      virtualFrame.getTransformToDesiredFrame(getTransformToParent(), ReferenceFrame.getWorldFrame());

      super.update();
   }

   public RigidBodyTransform getMarkerToWorld()
   {
      return markerToWorld;
   }

   public OpenCVArUcoMarker getArUcoMarker()
   {
      return arUcoMarker;
   }

   public ReferenceFrame getVirtualFrame()
   {
      return virtualFrame;
   }
}
