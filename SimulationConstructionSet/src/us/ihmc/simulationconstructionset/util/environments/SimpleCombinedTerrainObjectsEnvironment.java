package us.ihmc.simulationconstructionset.util.environments;

import java.awt.Color;

import javax.vecmath.Color3f;
import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.graphicsDescription.appearance.YoAppearanceMaterial;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class SimpleCombinedTerrainObjectsEnvironment extends CombinedTerrainObject3D
{
   public SimpleCombinedTerrainObjectsEnvironment()
   {
      super("SimpleCombinedTerrainObjectsEnvironment");


      // Add Terrain Objects

      // Rotated Table 1
      RigidBodyTransform configuration = new RigidBodyTransform();
      configuration.setRotationEulerAndZeroTranslation(0.0, 0.0, Math.toRadians(45.0));
      configuration.setTranslation(new Vector3d(4.0, 4.0, 0.7));
      this.addRotatableTable(configuration, 4.0, 2.0, 1.6, 0.1);

      // Table 2
      this.addTable(-1, -1, 3, 1, 1.3, 1.4);

      // Box 1
      this.addBox(-3.0, 2.0, -1.7, 0.5, 1.0);

      Color color = new Color(180, 220, 240);




      YoAppearanceMaterial appearance = new YoAppearanceMaterial();
      appearance.setDiffuseColor(new Color3f(color));
      appearance.setSpecularColor(new Color3f(color));
      appearance.setShininess(5.0f);

      // Rotated Box
      configuration.setRotationEulerAndZeroTranslation(new Vector3d(0.0, Math.toRadians(45.0), Math.toRadians(45.0)));
      configuration.setTranslation(new Vector3d(0.0, 3.5, 0.7));
      this.addRotatableBox(configuration, 4.0, 2.0, 1.6, appearance);

      // Ramp 1
      this.addRamp(-2.0, -4.0, 4.0, -2.0, 2.0);

      // Ramp 1
      this.addCone(6.0, 1.0, 2.0, 0.5, 1.0);

   }
}
