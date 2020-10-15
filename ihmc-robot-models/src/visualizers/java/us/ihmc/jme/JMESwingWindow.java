package us.ihmc.jme;

import com.jme3.asset.AssetManager;
import com.jme3.bounding.BoundingSphere;
import com.jme3.material.Material;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Sphere;
import javafx.scene.paint.Color;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple3D.Point3D;

import javax.swing.*;
import javax.swing.border.BevelBorder;
import java.awt.*;

public class JMESwingWindow
{
   private final JMECanvasApplication jmeCanvasApplication;
   private AssetManager assetManager;
   private Node rootNode;

   public JMESwingWindow()
   {
      jmeCanvasApplication = new JMECanvasApplication();

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
      assetManager = jmeCanvasApplication.getJME().getAssetManager();
      rootNode = jmeCanvasApplication.getZUpNode();

      jmeCanvasApplication.setupConstantColorSky(Color.GRAY);
      jmeCanvasApplication.setupPointLighting();
      jmeCanvasApplication.createCoordinateFrame(0.3);

      addDoor();

      rootNode.attachChild(addNormalSphere(0.3, 1.0, -1.0, 1.0, Color.RED));
//      rootNode.attachChild(addMeshBuilderSphere(0.3, 1.0, -1.0, 1.0, Color.RED));
   }

   private void addDoor()
   {
      Node doorNode = new Node();

      Spatial doorModel = assetManager.loadModel("models/door.obj");

      AxisAngle axisAngle = new AxisAngle(Axis3D.Y, Math.PI);

      us.ihmc.euclid.tuple4D.Quaternion q = new us.ihmc.euclid.tuple4D.Quaternion(axisAngle);

      Quaternion quaternion = new Quaternion();
      quaternion.set(q.getX32(), q.getY32(), q.getZ32(), q.getS32());
      doorModel.setLocalRotation(quaternion);

      doorModel.setLocalTranslation(0.921900f, 0f, 2.043600f);

      doorNode.attachChild(doorModel);

      axisAngle = new AxisAngle(Axis3D.Z, Math.PI / 2.0);
      q = new us.ihmc.euclid.tuple4D.Quaternion(axisAngle);
      quaternion.set(q.getX32(), q.getY32(), q.getZ32(), q.getS32());
      doorNode.setLocalRotation(quaternion);

      rootNode.attachChild(doorNode);

//      Geometry teaGeom = (Geometry) assetManager.loadModel("Models/Teapot/Teapot.obj");
//      teaGeom.setLocalTranslation(1.0f, 1.0f, 1.0f);
//      rootNode.attachChild(teaGeom);

//      Spatial object3DSGraphics = JME3DLoaderUtils.load3DModel("models/door.obj", assetManager, Graphics3DNodeType.VISUALIZATION);
//      rootNode.attachChild(object3DSGraphics);
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
      material.setColor("Diffuse", JMEColorConversions.toJMEColor(color));
      geometry.setMaterial(material);
      geometry.setLocalTranslation((float) x, (float) y, (float) z);
      return geometry;
   }

   public static void main(String[] args)
   {
      new JMESwingWindow();
   }
}