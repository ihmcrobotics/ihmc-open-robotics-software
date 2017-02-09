package us.ihmc.jMonkeyEngineToolkit.jme;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddMeshDataInstruction;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEAssetLocator;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEGraphicsObject;
import us.ihmc.robotics.Axis;

import com.jme3.app.SimpleApplication;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import com.jme3.system.AppSettings;

public class JMEChangeMeshTester extends SimpleApplication
{
   Graphics3DAddMeshDataInstruction instruction;
   private Node node;

   int counter = 0;
   @Override
   public void simpleInitApp()
   {

      Graphics3DObject graphics = new Graphics3DObject();
      graphics.setChangeable(true);
      graphics.rotate(Math.PI/2.0, Axis.Z);
      int resolution = 24;
      instruction = new Graphics3DAddMeshDataInstruction(MeshDataGenerator.Ellipsoid(1.0, 1.0, 1.0, resolution, resolution), YoAppearance.Red());
      JMEGraphicsObject graphicsObject = new JMEGraphicsObject(this, new JMEAssetLocator(assetManager), graphics);

      node = graphicsObject.getNode();
      rootNode.attachChild(node);

      flyCam.setDragToRotate(true);
      setupLighting();
   }
   private DirectionalLight setupDirectionalLight(Vector3f direction)
   {
      DirectionalLight d2 = new DirectionalLight();
      d2.setColor(ColorRGBA.White.mult(0.6f));
      Vector3f lightDirection2 = direction.normalizeLocal();
      d2.setDirection(lightDirection2);

      return d2;
   }

   private void setupLighting()
   {
      DirectionalLight primaryLight = setupDirectionalLight(new Vector3f(-0.5f, -8, -2));
      rootNode.addLight(primaryLight);

      DirectionalLight d2 = setupDirectionalLight(new Vector3f(1, -1, 1));
      rootNode.addLight(d2);

      DirectionalLight d3 = setupDirectionalLight(new Vector3f(1, -1, -1));
      rootNode.addLight(d3);

      DirectionalLight d4 = setupDirectionalLight(new Vector3f(-1, -1, 1));
      rootNode.addLight(d4);

      AmbientLight a1 = new AmbientLight();
      a1.setColor(ColorRGBA.White.mult(0.4f));
      rootNode.addLight(a1);

   }

   @Override
   public void simpleUpdate(float tpf)
   {
      if(counter % 1000 == 0)
      {
         instruction.setMesh(MeshDataGenerator.Cube(1.0, 1.0, 1.0,false, null));
         instruction.setAppearance(YoAppearance.Yellow());
      }
      else if (counter % 500 == 0)
      {
         instruction.setMesh(MeshDataGenerator.Sphere(1.0, 15, 15));
         instruction.setAppearance(YoAppearance.Green());
      }
      counter++;
   }

   public static void main(String[] args)
   {
      JMEChangeMeshTester jmeChangeMeshTester = new JMEChangeMeshTester();
      AppSettings appSettings = new AppSettings(true);
      appSettings.setResolution(800, 600);
      jmeChangeMeshTester.setSettings(appSettings);


      jmeChangeMeshTester.setShowSettings(false);
      jmeChangeMeshTester.setPauseOnLostFocus(false);

      jmeChangeMeshTester.start();
   }

}
