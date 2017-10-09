package us.ihmc.exampleSimulations.multilevelGround;

import us.ihmc.euclid.geometry.Box3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;

public class MarbleRunGround extends CombinedTerrainObject3D
{

   public MarbleRunGround(String name)
   {
      super(name);
      
      double heightDifferenceBetweenRamps = 0.5;
      
      Box3D box = new Box3D(1.0, 0.5, 0.1);
      box.setPosition(new Point3D(0.0, 0.0, 2.3));
      box.setOrientationYawPitchRoll(0.0, 0.5, 0.0);
      this.addRotatableBox(box, YoAppearance.Red());
      
      Box3D box2 = new Box3D(1.0, 0.5, 0.1);
      box2.setPosition(new Point3D(0.7, 0.0, 1.6));
      box2.setOrientationYawPitchRoll(0.0, -0.5, 0.0);
      this.addRotatableBox(box2, YoAppearance.Green());
      
      Box3D box3 = new Box3D(1.0, 0.5, 0.1);
      box3.setPosition(new Point3D(0.0, 0.0, 0.9));
      box3.setOrientationYawPitchRoll(0.0, 0.5, 0.0);
      this.addRotatableBox(box3, YoAppearance.Purple());
      
      Box3D box4 = new Box3D(1.0, 0.5, 0.1);
      box4.setPosition(new Point3D(0.7, 0.0, 0.2));
      box4.setOrientationYawPitchRoll(0.0, -0.5, 0.0);
      this.addRotatableBox(box4, YoAppearance.Gold());
      
      Box3D box5 = new Box3D(1.0, 0.5, 0.1);
      box5.setPosition(new Point3D(0.0, 0.0, -0.5));
      box5.setOrientationYawPitchRoll(0.0, 0.5, 0.0);
      this.addRotatableBox(box5, YoAppearance.BlueViolet());
   }

 
}
