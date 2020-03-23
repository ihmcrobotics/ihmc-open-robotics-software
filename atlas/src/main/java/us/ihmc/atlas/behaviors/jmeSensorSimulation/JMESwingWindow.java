package us.ihmc.atlas.behaviors.jmeSensorSimulation;

import com.jme3.asset.AssetManager;
import com.jme3.bounding.BoundingSphere;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.shape.Sphere;
import javafx.scene.paint.Color;
import us.ihmc.euclid.tuple3D.Point3D;

import javax.swing.*;
import javax.swing.border.BevelBorder;
import java.awt.*;

public class JMESwingWindow
{
   private final JMECanvasApplication jmeCanvasApplication;
   private final AssetManager assetManager;
   private final Node rootNode;

   public JMESwingWindow()
   {
      jmeCanvasApplication = new JMECanvasApplication();
      assetManager = jmeCanvasApplication.getJME().getAssetManager();
      rootNode = jmeCanvasApplication.getZUpNode();

      jmeCanvasApplication.enqueue(this::setupJME);

      JFrame frame = new JFrame("JME");
      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

      Container contentPane = frame.getContentPane();
      contentPane.setLayout(new BorderLayout());

      JPanel centerPanel = new JPanel(new BorderLayout());
      centerPanel.setBorder(BorderFactory.createBevelBorder(BevelBorder.LOWERED));
      centerPanel.setPreferredSize(new Dimension(1100, 800));
      centerPanel.add(jmeCanvasApplication.getCanvas(), BorderLayout.CENTER);
      contentPane.add(centerPanel, BorderLayout.CENTER);

      frame.pack();
      frame.setLocationByPlatform(true);
      frame.setVisible(true);
   }

   private void setupJME()
   {
      jmeCanvasApplication.setupConstantColorSky(Color.GRAY);
      jmeCanvasApplication.setupPointLighting();
      jmeCanvasApplication.createCoordinateFrame();
   }

   private Geometry addMeshBuilderSphere(double radius, double x, double y, double z, javafx.scene.paint.Color color)
   {
      JMEMultiColorMeshBuilder colorMeshBuilder = new JMEMultiColorMeshBuilder();
      colorMeshBuilder.addSphere((float) radius, new Point3D(x, y, z), color);
      Geometry geometry = new Geometry("meep", colorMeshBuilder.generateMesh());
      geometry.setMaterial(colorMeshBuilder.generateMaterial(assetManager));
      geometry.setModelBound(new BoundingSphere(Float.POSITIVE_INFINITY, Vector3f.ZERO));
      return geometry;
   }

   private Geometry addNormalSphere(double radius, double x, double y, double z, Color color)
   {
      Sphere sphere = new Sphere(10, 10, (float) radius);
      Geometry geometry = new Geometry("meep2", sphere);
      Material material = new Material(assetManager, "Common/MatDefs/Light/Lighting.j3md");
      material.setColor("Diffuse", new ColorRGBA((float) color.getRed(), (float) color.getGreen(), (float) color.getBlue(), 1.0f));
      geometry.setMaterial(material);
      geometry.setLocalTranslation((float) x, (float) y, (float) z);
      return geometry;
   }

   public static void main(String[] args)
   {
      new JMESwingWindow();
   }
}
