package us.ihmc.quadrupedRobotics.environments;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;

public class SimpleMazeEnvironment extends CombinedTerrainObject3D
{
   public SimpleMazeEnvironment()
   {
      super("SimpleMazeEnvironment");

      // ground
      RotatableBoxTerrainObject ground = new RotatableBoxTerrainObject(new Box3D(new Point3D(2.0, 0.0, -0.5), new Quaternion(), 6, 4, 1), YoAppearance.Grey());
      addTerrainObject(ground);

      // front-left wall
      RotatableBoxTerrainObject wall1 = new RotatableBoxTerrainObject(new Box3D(new Point3D(1.5, 0.75, 0.5), new Quaternion(), 0.1, 2.5, 1.0), YoAppearance.DarkGrey());
      addTerrainObject(wall1);

      // front-right wall
      RotatableBoxTerrainObject wall2 = new RotatableBoxTerrainObject(new Box3D(new Point3D(1.5, -1.625, 0.5), new Quaternion(), 0.1, 0.75, 1.0), YoAppearance.DarkGrey());
      addTerrainObject(wall2);

      // back-left wall
      RotatableBoxTerrainObject wall3 = new RotatableBoxTerrainObject(new Box3D(new Point3D(3.0, 1.625, 0.5), new Quaternion(), 0.1, 0.75, 1.0), YoAppearance.DarkGrey());
      addTerrainObject(wall3);

      // back-right wall
      RotatableBoxTerrainObject wall4 = new RotatableBoxTerrainObject(new Box3D(new Point3D(3.0, -0.75, 0.5), new Quaternion(), 0.1, 2.5, 1.0), YoAppearance.DarkGrey());
      addTerrainObject(wall4);
   }

   @Override
   public double heightAt(double x, double y, double z)
   {
      return 0.0;
   }

   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3D normalToPack)
   {
      normalToPack.setZ(1.0);
      return 0.0;
   }

   @Override
   public BoundingBox3D getBoundingBox()
   {
      return new BoundingBox3D(-10.0, -10.0, -10.0, 10.0, 10.0, 10.0);
   }
}
