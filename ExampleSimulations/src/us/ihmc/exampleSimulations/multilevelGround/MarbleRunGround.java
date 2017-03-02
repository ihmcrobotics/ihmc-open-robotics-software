package us.ihmc.exampleSimulations.multilevelGround;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.shapes.Box3d;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;

public class MarbleRunGround extends CombinedTerrainObject3D
{

   public MarbleRunGround(String name)
   {
      super(name);
      
      double heightDifferenceBetweenRamps = 0.5;
      
      Box3d box = new Box3d(1.0, 0.5, 0.1);
      box.setPosition(new Point3D(0.0, 0.0, 2.3));
      box.setYawPitchRoll(0.0, 0.5, 0.0);
      this.addRotatableBox(box, YoAppearance.Red());
      
      Box3d box2 = new Box3d(1.0, 0.5, 0.1);
      box2.setPosition(new Point3D(0.7, 0.0, 1.6));
      box2.setYawPitchRoll(0.0, -0.5, 0.0);
      this.addRotatableBox(box2, YoAppearance.Green());
      
      Box3d box3 = new Box3d(1.0, 0.5, 0.1);
      box3.setPosition(new Point3D(0.0, 0.0, 0.9));
      box3.setYawPitchRoll(0.0, 0.5, 0.0);
      this.addRotatableBox(box3, YoAppearance.Purple());
      
      Box3d box4 = new Box3d(1.0, 0.5, 0.1);
      box4.setPosition(new Point3D(0.7, 0.0, 0.2));
      box4.setYawPitchRoll(0.0, -0.5, 0.0);
      this.addRotatableBox(box4, YoAppearance.Gold());
      
      Box3d box5 = new Box3d(1.0, 0.5, 0.1);
      box5.setPosition(new Point3D(0.0, 0.0, -0.5));
      box5.setYawPitchRoll(0.0, 0.5, 0.0);
      this.addRotatableBox(box5, YoAppearance.BlueViolet());
   }

 
}
