package us.ihmc.jMonkeyEngineToolkit.jme.lidar;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.Sphere3D;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.structure.Graphics3DNodeType;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DWorld;

public class Graphics3DLidarScan
{
   private static final double SPHERE_RADIUS = 0.005;

   private Graphics3DWorld world;
   private String lidarName;
   private LidarTestParameters params;
   private AppearanceDefinition appearance;

   private Graphics3DNode[] points;
   private Graphics3DNode[] rays;

   public Graphics3DLidarScan(Graphics3DWorld world, String lidarName, LidarTestParameters params, AppearanceDefinition appearance)
   {
      this.world = world;
      this.lidarName = lidarName;
      this.params = params;
      this.appearance = appearance;

      init();
   }

   private void init()
   {
      if (params.getShowTracePoints())
      {
         createPoints();
         world.addAllChildren(points);
      }

      if (params.getShowScanRays())
      {
         createRays();
         world.addAllChildren(rays);
      }
   }

   public void update(LidarTestScan lidarScan)
   {
      for (int i = 0; (i < lidarScan.size()) && (i < params.getScansPerSweep()); i++)
      {
         if (params.getShowTracePoints())
         {
            if ((lidarScan.getRange(i) < params.getMinRange()) || (lidarScan.getRange(i) > params.getMaxRange()))
            {
               points[i].setTransform(new AffineTransform());
            }
            else
            {
               RigidBodyTransform pointTransform = new RigidBodyTransform();

               Point3D p = new Point3D(lidarScan.getRange(i) + (SPHERE_RADIUS * 1.1), 0.0, 0.0);
               RigidBodyTransform transform = new RigidBodyTransform();
               lidarScan.getInterpolatedTransform(i, transform);
               transform.multiply(RayTracingLidar.getSweepTransform(params, i));
               transform.transform(p);

               pointTransform.setTranslation(new Vector3D32(p));

               points[i].setTransform(pointTransform);
            }
         }

         if (params.getShowScanRays())
         {
            RigidBodyTransform rayTransform = new RigidBodyTransform();
            lidarScan.getInterpolatedTransform(i, rayTransform);
            rayTransform.multiply(RayTracingLidar.getSweepTransform(params, i));
            rays[i].setTransform(rayTransform);
         }
      }
   }

   private void createPoints()
   {
      int scansPerSweep = params.getScansPerSweep();
      points = new Graphics3DNode[scansPerSweep];

      for (int i = 0; i < scansPerSweep; i++)
      {
         points[i] = new Graphics3DNode(lidarName + "point" + i, Graphics3DNodeType.VISUALIZATION,
                                        new Graphics3DObject(new Sphere3D(SPHERE_RADIUS), appearance));
      }
   }

   private void createRays()
   {
      int scansPerSweep = params.getScansPerSweep();
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
