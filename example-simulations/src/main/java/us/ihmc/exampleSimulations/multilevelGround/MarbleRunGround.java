package us.ihmc.exampleSimulations.multilevelGround;

import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;

public class MarbleRunGround extends CombinedTerrainObject3D
{

   public MarbleRunGround(String name)
   {
      super(name);
      
      double heightDifferenceBetweenRamps = 0.5;
      
      Box3D box = new Box3D(1.0, 0.5, 0.1);
      box.getPosition().set(new Point3D(0.0, 0.0, 2.3));
      box.getOrientation().setYawPitchRoll(0.0, 0.5, 0.0);
      this.addRotatableBox(box, YoAppearance.Red());
      
      Box3D box2 = new Box3D(1.0, 0.5, 0.1);
      box2.getPosition().set(new Point3D(0.7, 0.0, 1.6));
      box2.getOrientation().setYawPitchRoll(0.0, -0.5, 0.0);
      this.addRotatableBox(box2, YoAppearance.Green());
      
      Box3D box3 = new Box3D(1.0, 0.5, 0.1);
      box3.getPosition().set(new Point3D(0.0, 0.0, 0.9));
      box3.getOrientation().setYawPitchRoll(0.0, 0.5, 0.0);
      this.addRotatableBox(box3, YoAppearance.Purple());
      
      Box3D box4 = new Box3D(1.0, 0.5, 0.1);
      box4.getPosition().set(new Point3D(0.7, 0.0, 0.2));
      box4.getOrientation().setYawPitchRoll(0.0, -0.5, 0.0);
      this.addRotatableBox(box4, YoAppearance.Gold());
      
      Box3D box5 = new Box3D(1.0, 0.5, 0.1);
      box5.getPosition().set(new Point3D(0.0, 0.0, -0.5));
      box5.getOrientation().setYawPitchRoll(0.0, 0.5, 0.0);
      this.addRotatableBox(box5, YoAppearance.BlueViolet());
   }

 
}
