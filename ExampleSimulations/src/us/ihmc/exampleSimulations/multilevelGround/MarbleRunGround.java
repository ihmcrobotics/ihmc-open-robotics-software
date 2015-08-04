package us.ihmc.exampleSimulations.multilevelGround;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.robotics.geometry.shapes.Box3d;

public class MarbleRunGround extends CombinedTerrainObject3D
{

   public MarbleRunGround(String name)
   {
      super(name);
      
      double heightDifferenceBetweenRamps = 0.5;
      
      Box3d box = new Box3d(1.0, 0.5, 0.1);
      box.setTranslation(new Vector3d(0.0, 0.0, 2.3));
      box.setYawPitchRoll(0.0, 0.5, 0.0);
      this.addRotatableBox(box, YoAppearance.Red());
      
      Box3d box2 = new Box3d(1.0, 0.5, 0.1);
      box2.setTranslation(new Vector3d(0.7, 0.0, 1.6));
      box2.setYawPitchRoll(0.0, -0.5, 0.0);
      this.addRotatableBox(box2, YoAppearance.Green());
      
      Box3d box3 = new Box3d(1.0, 0.5, 0.1);
      box3.setTranslation(new Vector3d(0.0, 0.0, 0.9));
      box3.setYawPitchRoll(0.0, 0.5, 0.0);
      this.addRotatableBox(box3, YoAppearance.Purple());
      
      Box3d box4 = new Box3d(1.0, 0.5, 0.1);
      box4.setTranslation(new Vector3d(0.7, 0.0, 0.2));
      box4.setYawPitchRoll(0.0, -0.5, 0.0);
      this.addRotatableBox(box4, YoAppearance.Gold());
      
      Box3d box5 = new Box3d(1.0, 0.5, 0.1);
      box5.setTranslation(new Vector3d(0.0, 0.0, -0.5));
      box5.setYawPitchRoll(0.0, 0.5, 0.0);
      this.addRotatableBox(box5, YoAppearance.BlueViolet());
   }

 
}
