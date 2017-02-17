package us.ihmc.jMonkeyEngineToolkit.utils.lidar;

import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.structure.Graphics3DNodeType;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DWorld;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.shapes.Sphere3d;
import us.ihmc.robotics.lidar.LidarScan;

public class Graphics3DLidarScan
{
   private static final double SPHERE_RADIUS = 0.005;

   private Graphics3DWorld world;
   private String lidarName;
   private int scansPerSweep;
   private double minRange;
   private double maxRange;
   private AppearanceDefinition appearance;
   private boolean showScanRays;
   private boolean showScanPoints;

   private Graphics3DNode[] points;
   private Graphics3DNode[] rays;

   public Graphics3DLidarScan(Graphics3DWorld world, String lidarName, int scansPerSweep, double minRange, double maxRange, boolean showScanRays,
                             boolean showScanPoints, AppearanceDefinition appearance)
   {
      this.world = world;
      this.lidarName = lidarName;
      this.scansPerSweep = scansPerSweep;
      this.minRange = minRange;
      this.maxRange = maxRange;
      this.showScanRays = showScanRays;
      this.showScanPoints = showScanPoints;
      this.appearance = appearance;

      init();
   }

   private void init()
   {
      if (showScanPoints)
      {
         createPoints();
         world.addAllChildren(points);
      }

      if (showScanRays)
      {
         createRays();
         world.addAllChildren(rays);
      }
   }

   public void update(LidarScan lidarScan)
   {
      for (int i = 0; (i < lidarScan.size()) && (i < scansPerSweep); i++)
      {
         if (showScanPoints)
         {
            if ((lidarScan.getRange(i) < minRange) || (lidarScan.getRange(i) > maxRange))
            {
               points[i].setTransform(new AffineTransform());
            }
            else
            {
               RigidBodyTransform pointTransform = new RigidBodyTransform();

               Point3D p = new Point3D(lidarScan.getRange(i) + (SPHERE_RADIUS * 1.1), 0.0, 0.0);
               RigidBodyTransform transform = new RigidBodyTransform();
               lidarScan.getInterpolatedTransform(i, transform);
               transform.multiply(lidarScan.getSweepTransform(i));
               transform.transform(p);

               pointTransform.setTranslation(new Vector3D32(p));

               points[i].setTransform(pointTransform);
            }
         }

         if (showScanRays)
         {
            RigidBodyTransform rayTransform = new RigidBodyTransform();
            lidarScan.getInterpolatedTransform(i, rayTransform);
            rayTransform.multiply(lidarScan.getSweepTransform(i));
            rays[i].setTransform(rayTransform);
         }
      }
   }

   private void createPoints()
   {
      points = new Graphics3DNode[scansPerSweep];

      for (int i = 0; i < scansPerSweep; i++)
      {
         points[i] = new Graphics3DNode(lidarName + "point" + i, Graphics3DNodeType.VISUALIZATION,
                                        new Graphics3DObject(new Sphere3d(SPHERE_RADIUS), appearance));
      }
   }

   private void createRays()
   {
      rays = new Graphics3DNode[scansPerSweep];

      for (int i = 0; i < scansPerSweep; i++)
      {
         Graphics3DObject rayObject = new Graphics3DObject();
         rayObject.rotate(Math.PI / 2, Axis.Y);
         rayObject.addCylinder(1.0, 0.0005, appearance);

         rays[i] = new Graphics3DNode(lidarName + "ray" + i, Graphics3DNodeType.VISUALIZATION, rayObject);
      }
   }
}
