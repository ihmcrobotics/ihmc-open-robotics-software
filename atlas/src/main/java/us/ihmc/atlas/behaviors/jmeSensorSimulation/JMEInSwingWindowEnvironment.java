package us.ihmc.atlas.behaviors.jmeSensorSimulation;

import com.jme3.asset.AssetConfig;
import com.jme3.asset.TextureKey;
import com.jme3.bounding.BoundingSphere;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.light.PointLight;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.material.TechniqueDef;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.ViewPort;
import com.jme3.renderer.opengl.GLRenderer;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.plugins.ogre.MaterialLoader;
import com.jme3.scene.shape.Box;
import com.jme3.scene.shape.Sphere;
import com.jme3.system.AppSettings;
import com.jme3.system.JmeCanvasContext;
import com.jme3.system.JmeSystem;
import com.jme3.system.lwjgl.LwjglContext;
import com.jme3.texture.Texture;
import com.jme3.texture.plugins.AWTLoader;
import com.jme3.util.SkyFactory;
import javafx.scene.paint.Color;
import jme3dae.ColladaLoader;
import jme3dae.collada14.ColladaDocumentV14;
import jme3dae.materials.FXBumpMaterialGenerator;
import jme3tools.optimize.GeometryBatchFactory;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEGeometryUtils;
import us.ihmc.jMonkeyEngineToolkit.stlLoader.STLLoader;
import us.ihmc.log.LogTools;

import javax.swing.*;
import javax.swing.border.BevelBorder;
import java.awt.*;
import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;

import static java.util.logging.Logger.*;

public class JMEInSwingWindowEnvironment
{
   private static final boolean DISABLE_LOGGING = false;
   private final java.util.logging.Logger[] jmeLoggers = new java.util.logging.Logger[] {getLogger(FXBumpMaterialGenerator.class.getName()),
                                                                                         getLogger(ColladaDocumentV14.class.getName()),
                                                                                         getLogger(GLRenderer.class.getName()),
                                                                                         getLogger(AssetConfig.class.getName()),
                                                                                         getLogger(JmeSystem.class.getName()),
                                                                                         getLogger(LwjglContext.class.getName())};

   private final FunctionalSimpleApplication jme = new FunctionalSimpleApplication();

   private Node zUpNode;
   private Node floor;
   private FocusBasedJMECamera customCamera;

   public JMEInSwingWindowEnvironment()
   {
      if (DISABLE_LOGGING)
      {
         for (Logger jmeLogger : jmeLoggers)
         {
            jmeLogger.setLevel(Level.SEVERE);
         }
      }

      AppSettings appSettings = new AppSettings(true);

//      appSettings.setCustomRenderer(PBOAwtPanelsContext.class); //??

      appSettings.setAudioRenderer(null);
      appSettings.setResolution(1100, 800);
      appSettings.setVSync(true);
      appSettings.setSamples(4);

      jme.setSimpleInitApp(this::simpleInitApp);
      jme.setInitialize(this::initialize);
      jme.setSimpleUpdate(this::simpleUpdate);
      jme.setPauseOnLostFocus(false);
      jme.setShowSettings(false);
      jme.setSettings(appSettings);
      jme.setDisplayFps(false);
      jme.setDisplayStatView(true);

      jme.createCanvas();


//      jmeCanvasContext.getCanvas().setPreferredSize(new Dimension(1600, 600));

      // AWTPanelsContextManager ?

      jme.startCanvas();
//      jme.start();


   }

   private void simpleInitApp()
   {
      LogTools.info("Hello JME");

      Thread.currentThread().setUncaughtExceptionHandler(null);
      Thread.setDefaultUncaughtExceptionHandler(null);

      jme.getAssetManager().registerLoader(AWTLoader.class, "tif");
      jme.getAssetManager().registerLoader(ColladaLoader.class, "dae");
      jme.getAssetManager().registerLoader(STLLoader.class, "stl");
      jme.getAssetManager().registerLoader(MaterialLoader.class, "material");

      zUpNode = new Node("zUpNode");
      zUpNode.setLocalRotation(JMEGeometryUtils.getRotationFromJMEToZupCoordinates());
      jme.getRootNode().attachChild(zUpNode);



      jme.getFlyByCamera().setEnabled(false);
      jme.getViewPort().setEnabled(false);
      jme.getRenderManager().removeMainView(jme.getViewPort());
//      jme.getRenderManager().removeMainView(jme.getGuiViewPort());

//      jme.getRenderManager().
//      jme.getRenderer().

      customCamera = new FocusBasedJMECamera(1100, 800, jme.getInputManager());

      ViewPort customViewport = jme.getRenderManager().createMainView("JMEViewport", customCamera);
      customViewport.attachScene(jme.getRootNode());
      customViewport.setClearFlags(true, true, true);
      customViewport.setBackgroundColor(new ColorRGBA(0.5019608f, 0.5019608f, 0.5019608f, 1.0f));

      setupSky();
      setupLighting2();
      setupLighting();

      createCoordinateFrame();

      zUpNode.attachChild(addSphere2(0.1, 1.0, 1.0, 1.0, Color.RED));
      zUpNode.attachChild(addSphere2(0.1, -1.0, 1.0, 1.0, Color.RED));
      zUpNode.attachChild(addSphere2(0.1, -1.0, -1.0, 1.0, Color.RED));
      zUpNode.attachChild(addSphere2(0.1, 1.0, -1.0, 1.0, Color.RED));
      zUpNode.attachChild(addSphere2(0.1, 1.0, 1.0, -1.0, Color.RED));
      zUpNode.attachChild(addSphere2(0.1, -1.0, 1.0, -1.0, Color.RED));
      zUpNode.attachChild(addSphere2(0.1, -1.0, -1.0, -1.0, Color.RED));
      zUpNode.attachChild(addSphere2(0.1, 1.0, -1.0, -1.0, Color.RED));
      zUpNode.attachChild(addSkySphere(50.0, 0.0, 0.0, 0.0, Color.WHITE));
      zUpNode.attachChild(addSphere(0.3, -1.0, -1.0, -1.0, Color.BLUE));


      RenderState renderState = new RenderState();
      renderState.setDepthWrite(false);
      renderState.setDepthFunc(RenderState.TestFunction.Equal);
      jme.getRenderer().applyRenderState(renderState);

//      jme.getRenderer().getStatistics().
//      setUpGrid();

      JFrame frame = new JFrame("JME");
      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

      Container contentPane = frame.getContentPane();
      contentPane.setLayout(new BorderLayout());

      JPanel centerPanel = new JPanel(new BorderLayout());
      centerPanel.setBorder(BorderFactory.createBevelBorder(BevelBorder.LOWERED));
      centerPanel.setPreferredSize(new Dimension(1100, 800));
      centerPanel.add(((JmeCanvasContext) jme.getContext()).getCanvas(), BorderLayout.CENTER);
      contentPane.add(centerPanel, BorderLayout.CENTER);

      frame.pack();
      frame.setLocationByPlatform(true);
      frame.setVisible(true);
   }

   private void createCoordinateFrame()
   {
      double length = 1.0;
      double radius = 0.02 * length;
      double coneHeight = 0.10 * length;
      double coneRadius = 0.05 * length;
      Node coordinateSystemNode = new Node();
      int axisSamples = 10;
      int radialSamples = 1;
      boolean closed = true;

      JMEMultiColorMeshBuilder meshBuilder = new JMEMultiColorMeshBuilder();
      meshBuilder.addCylinder(length, radius, new Point3D(), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), Color.RED);
      meshBuilder.addCone(coneHeight, coneRadius, new Point3D(length, 0.0, 0.0), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), Color.RED);
      meshBuilder.addCylinder(length, radius, new Point3D(), new AxisAngle(1.0, 0.0, 0.0, -Math.PI / 2.0), Color.GREEN);
      meshBuilder.addCone(coneHeight, coneRadius, new Point3D(0.0, length, 0.0), new AxisAngle(1.0, 0.0, 0.0, -Math.PI / 2.0), Color.GREEN);
      meshBuilder.addCylinder(length, radius, new Point3D(), new AxisAngle(), Color.BLUE);
      meshBuilder.addCone(coneHeight, coneRadius, new Point3D(0.0, 0.0, length), new AxisAngle(), Color.BLUE);
      Mesh mesh = meshBuilder.generateMesh();

      Geometry geometry = new Geometry("g1", mesh);
      geometry.setMaterial(meshBuilder.generateMaterial(jme.getAssetManager()));


////      Cylinder cylinder = new Cylinder(axisSamples, radialSamples, (float) radius, (float) length, closed);
//      Material blueMaterial = new Material(jme.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
////      blueMaterial.setColor("Color", ColorRGBA.Blue);
////      Texture texture = new Texture();
//      Texture texture = jme.getAssetManager().loadTexture("palette.png");
////      Texture texture = jme.getAssetManager().loadTexture("running-man-32x32.png");
//      blueMaterial.setTexture("ColorMap", texture);
//      geometry.setMaterial(blueMaterial);

//      zUpNode.attachChild(geometry);
      zUpNode.attachChild(geometry);
//      cylinderGeometry.

   }

//   private MeshDataHolder setColor(MeshDataHolder input, Color color)
//   {
//      if (input == null)
//         return null;
//      Point3D32[] vertices = input.getVertices();
//      int[] triangleIndices = input.getTriangleIndices();
//      Vector3D32[] vertexNormals = input.getVertexNormals();
//      TexCoord2f[] inputTexturePoints = input.getTexturePoints();
//      TexCoord2f[] outputTexturePoints = new TexCoord2f[inputTexturePoints.length];
//      float[] textureLocation = colorPalette.getTextureLocation(color);
//      for (int i = 0; i < inputTexturePoints.length; i++)
//         outputTexturePoints[i] = new TexCoord2f(textureLocation);
//      return new MeshDataHolder(vertices, outputTexturePoints, triangleIndices, vertexNormals);
//   }

   private void initialize()
   {

   }

   private void simpleUpdate(float tpf)
   {
      customCamera.simpleUpdate(tpf);
   }

   private void setupSky()
   {
      String west = "Textures/Sky/Bright/skyboxsun25degtest/skyrender0005.bmp";
      String east = "Textures/Sky/Bright/skyboxsun25degtest/skyrender0002.bmp";
      String north = "Textures/Sky/Bright/skyboxsun25degtest/skyrender0001.bmp";
      String south = "Textures/Sky/Bright/skyboxsun25degtest/skyrender0004.bmp";
      String up = "Textures/Sky/Bright/skyboxsun25degtest/skyrender0003.bmp";
      String down = "Textures/Sky/Bright/skyboxsun25degtest/skyrender0007.bmp";
      Texture westTex = jme.getAssetManager().loadTexture(new TextureKey(west, true));
      Texture eastTex = jme.getAssetManager().loadTexture(new TextureKey(east, true));
      Texture northTex = jme.getAssetManager().loadTexture(new TextureKey(north, true));
      Texture southTex = jme.getAssetManager().loadTexture(new TextureKey(south, true));
      Texture upTex = jme.getAssetManager().loadTexture(new TextureKey(up, true));
      Texture downTex = jme.getAssetManager().loadTexture(new TextureKey(down, true));
      Spatial sky = SkyFactory.createSky(jme.getAssetManager(), westTex, eastTex, northTex, southTex, upTex, downTex);
      sky.setLocalScale(1000);
//      zUpNode.attachChild(sky);
//      jme.getRootNode().attachChild(SkyFactory.createSky(jme.getAssetManager(), "Textures/Sky/Bright/BrightSky.dds", SkyFactory.EnvMapType.CubeMap));
   }

   private void setupLighting2()
   {
      float ambientValue = 0.7f;
//      float pointValue = 0.2f;
      float pointValue = 0.8f;
//      float pointDistance = 1000.0f;
      float pointDistance = 1.5f;

      ColorRGBA ambientColor = new ColorRGBA(ambientValue, ambientValue, ambientValue, 1.0f);
      zUpNode.addLight(new AmbientLight(ambientColor));
      ColorRGBA indoorColor = new ColorRGBA(pointValue, pointValue, pointValue, 1.0f);
//      zUpNode.addLight(new PointLight(new Vector3f(pointDistance, pointDistance, pointDistance), indoorColor));
//      zUpNode.addLight(new PointLight(new Vector3f(-pointDistance, pointDistance, pointDistance), indoorColor));
//      zUpNode.addLight(new PointLight(new Vector3f(-pointDistance, -pointDistance, pointDistance), indoorColor));
//      zUpNode.addLight(new PointLight(new Vector3f(pointDistance, -pointDistance, pointDistance), indoorColor));

       // order seems to be odd: y, z, x (but only when sky added)
//      zUpNode.addLight(new PointLight(new Vector3f(-pointDistance, -pointDistance, -pointDistance), indoorColor));
      zUpNode.addLight(new PointLight(new Vector3f(pointDistance, pointDistance, pointDistance), indoorColor, 100.0f));
      zUpNode.addLight(new PointLight(new Vector3f(-pointDistance, pointDistance, pointDistance), indoorColor, 100.0f));
//      zUpNode.addLight(new PointLight(new Vector3f(-pointDistance, pointDistance, -pointDistance), indoorColor, 100.0f));
//      zUpNode.addLight(new PointLight(new Vector3f(pointDistance, pointDistance, -pointDistance), indoorColor, 100.0f));
//      zUpNode.addLight(new PointLight(new Vector3f(pointDistance, -pointDistance, pointDistance), indoorColor, 100.0f));
//      zUpNode.addLight(new PointLight(new Vector3f(-pointDistance, -pointDistance, pointDistance), indoorColor, 100.0f));
//      zUpNode.addLight(new PointLight(new Vector3f(-pointDistance, -pointDistance, -pointDistance), indoorColor, 100.0f));
//      zUpNode.addLight(new PointLight(new Vector3f(pointDistance, -pointDistance, -pointDistance), indoorColor, 100.0f));

//      jme.getRootNode().addLight(new PointLight(new Vector3f(-pointDistance, pointDistance, pointDistance), indoorColor));
//      jme.getRootNode().addLight(new PointLight(new Vector3f(pointDistance, pointDistance, pointDistance), indoorColor));
//      jme.getRootNode().addLight(new PointLight(new Vector3f(-pointDistance, pointDistance, -pointDistance), indoorColor));
//      jme.getRootNode().addLight(new PointLight(new Vector3f(pointDistance, pointDistance, -pointDistance), indoorColor));

//      // order seems to be odd: y, z, x
//      zUpNode.addLight(new PointLight(new Vector3f(-pointDistance, pointDistance, -pointDistance), indoorColor));
////      zUpNode.addLight(new PointLight(new Vector3f(-pointDistance, pointDistance, pointDistance), indoorColor));
////      zUpNode.addLight(new PointLight(new Vector3f(-pointDistance, pointDistance, -pointDistance), indoorColor));
////      zUpNode.addLight(new PointLight(new Vector3f(pointDistance, pointDistance, -pointDistance), indoorColor));
   }

   private Geometry addSphere(double radius, double x, double y, double z, Color color)
   {
      JMEMultiColorMeshBuilder colorMeshBuilder = new JMEMultiColorMeshBuilder();
      colorMeshBuilder.addSphere((float) radius, new Point3D(x, y, z), color);
      Geometry geometry = new Geometry("meep", colorMeshBuilder.generateMesh());
      geometry.setMaterial(colorMeshBuilder.generateMaterial(jme.getAssetManager()));
//      geometry.setQueueBucket(RenderQueue.Bucket.Sky);
//      geometry.setCullHint(Spatial.CullHint.Never);
      geometry.setModelBound(new BoundingSphere(Float.POSITIVE_INFINITY, Vector3f.ZERO));
      return geometry;
   }

   private Geometry addSphere2(double radius, double x, double y, double z, Color color)
   {
      Sphere sphere = new Sphere(10, 10, (float) radius);

      Geometry geometry = new Geometry("meep2", sphere);
//      Material material = new Material(jme.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
      Material material = new Material(jme.getAssetManager(), "Common/MatDefs/Light/Lighting.j3md");
      material.setColor("Diffuse", new ColorRGBA((float) color.getRed(), (float) color.getGreen(), (float) color.getBlue(), 1.0f));
      geometry.setMaterial(material);
      geometry.setLocalTranslation((float) x, (float) y, (float) z);
      //      geometry.setQueueBucket(RenderQueue.Bucket.Sky);
      //      geometry.setCullHint(Spatial.CullHint.Never);
//      geometry.setModelBound(new BoundingSphere(Float.POSITIVE_INFINITY, Vector3f.ZERO));
      return geometry;
   }

   private Geometry addSkySphere(double radius, double x, double y, double z, Color color)
   {
      Sphere sphere = new Sphere(10, 10, (float) radius, false, false);

      Geometry geometry = new Geometry("meep2", sphere);
//      geometry.setQueueBucket(RenderQueue.Bucket.Sky);
//      geometry.setCullHint(Spatial.CullHint.Never);
//      geometry.setModelBound(new BoundingSphere(Float.POSITIVE_INFINITY, Vector3f.ZERO));
//      Material material = new Material(jme.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
      Material material = new Material(jme.getAssetManager(), "Common/MatDefs/Misc/Sky.j3md");
//      material.setVector3("NormalScale", Vector3f.UNIT_XYZ);
//      material.setColor("Diffuse", new ColorRGBA((float) color.getRed(), (float) color.getGreen(), (float) color.getBlue(), 1.0f));
//      material.setBoolean("SphereMap", true);
      geometry.setMaterial(material);
      geometry.setLocalTranslation((float) x, (float) y, (float) z);
      //      geometry.setQueueBucket(RenderQueue.Bucket.Sky);
      //      geometry.setCullHint(Spatial.CullHint.Never);
//      geometry.setModelBound(new BoundingSphere(Float.POSITIVE_INFINITY, Vector3f.ZERO));
      return geometry;
   }

   private void setupLighting()
   {
      // Note: Something gets transformed, so that z is x, and y is z, and x is y.

      Vector3f fromTheTop = new Vector3f(0.0f, -1.0f, 0.0f).normalizeLocal();
      Vector3f fromTheFront = new Vector3f(0.0f, 0.0f, -1.0f).normalizeLocal();
      Vector3f fromTheSide = new Vector3f(-1.0f, 0.0f, 0.0f).normalizeLocal();

      Vector3f fromTheBack = new Vector3f(fromTheFront).negate().normalizeLocal();
      Vector3f fromTheOtherSide = new Vector3f(fromTheSide).negate().normalizeLocal();
      Vector3f fromTheBottom = new Vector3f(fromTheTop).negate().normalizeLocal();

      Vector3f fromTheTopFront = fromTheTop.add(fromTheFront).normalizeLocal();
      Vector3f fromTheFrontSide = fromTheFront.add(fromTheSide).normalizeLocal();
      Vector3f fromTheBackOtherSide = fromTheBack.add(fromTheOtherSide).normalizeLocal();
      Vector3f fromTheBackBottom = fromTheBack.add(fromTheBottom).normalizeLocal();

      DirectionalLight primaryLight = new DirectionalLight();
      primaryLight.setColor(ColorRGBA.White.mult(0.5f));
      primaryLight.setDirection(fromTheTopFront);
//      jme.getRootNode().addLight(primaryLight);
      ArrayList<DirectionalLight> lights = new ArrayList<>();
      lights.add(primaryLight);

      // TODO: Ambient light could be brighter, but OBJ model files seem to just get white in ambient light, rather than their material.
      AmbientLight ambientLight = new AmbientLight();
      ambientLight.setColor(ColorRGBA.White.mult(0.2f));
//      jme.getRootNode().addLight(ambientLight);

//      addDirectionalLight(ColorRGBA.White.mult(0.35f), fromTheFrontSide, lights, primaryLight);
//      addDirectionalLight(ColorRGBA.White.mult(0.3f), fromTheBackOtherSide, lights, primaryLight);
//      addDirectionalLight(ColorRGBA.White.mult(0.28f), fromTheBackBottom, lights, primaryLight);
//      addDirectionalLight(ColorRGBA.White.mult(0.32f), fromTheOtherSide, lights, primaryLight);
//      addDirectionalLight(ColorRGBA.White.mult(0.35f), fromTheSide, lights, primaryLight);

//      jme.getRenderManager().setPreferredLightMode(TechniqueDef.LightMode.SinglePass);
      jme.getRenderManager().setPreferredLightMode(TechniqueDef.LightMode.MultiPass);

      jme.getRootNode().setShadowMode(RenderQueue.ShadowMode.CastAndReceive);
      zUpNode.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);
   }

   private void addDirectionalLight(ColorRGBA color, Vector3f direction, ArrayList<DirectionalLight> lights, DirectionalLight primaryLight)
   {
      if (lights.isEmpty())
      {
         primaryLight.setColor(color);
         primaryLight.setDirection(direction);
         lights.add(primaryLight);
         jme.getRootNode().addLight(primaryLight);
      }
      else
      {
         DirectionalLight light = new DirectionalLight();
         light.setColor(color);
         light.setDirection(direction.normalizeLocal());
         lights.add(light);
         jme.getRootNode().addLight(light);
      }
   }

   private void setUpGrid()
   {
      Node grid = new Node();
      float size = 200;
      float height = 0.0025f;
      float width = 0.02f;

      // draw base Axis Lines
      for (int x = new Float(-size / 2).intValue(); x < new Float(size / 2).intValue(); x++)
      {
         Vector3f centerBox2 = new Vector3f(x, 0, -height - 0.01f);
         Box box2 = new Box(0.0f, 0.0f, 0.0f);
         box2.updateGeometry(centerBox2, width, size / 2, height);

         Geometry line = new Geometry("Box2", box2);
         Material mat2 = new Material(jme.getAssetManager(), "Common/MatDefs/Light/Lighting.j3md");
         if (x == 0)
         {
            mat2.setBoolean("UseMaterialColors", true);

            mat2.setColor("Diffuse", ColorRGBA.DarkGray);
         }
         else
         {
            mat2.setBoolean("UseMaterialColors", true);

            mat2.setColor("Diffuse", ColorRGBA.Gray);
         }

         line.setShadowMode(RenderQueue.ShadowMode.Off);
         line.setMaterial(mat2);
         grid.attachChild(line);
      }

      for (float y = new Float(-size / 2).intValue(); y < new Float(size / 2).intValue(); y++)
      {
         Vector3f centerBox2 = new Vector3f(0, y, -height - 0.01f);
         Box box2 = new Box(0.0f, 0.0f, 0.0f);
         box2.updateGeometry(centerBox2, size / 2, width, height);

         Geometry line = new Geometry("Box2", box2);
         Material mat2 = new Material(jme.getAssetManager(), "Common/MatDefs/Light/Lighting.j3md");
         if (y == 0)
         {
            mat2.setBoolean("UseMaterialColors", true);

            mat2.setColor("Diffuse", ColorRGBA.DarkGray);
         }
         else
         {
            mat2.setBoolean("UseMaterialColors", true);

            mat2.setColor("Diffuse", ColorRGBA.Gray);
         }

         line.setMaterial(mat2);
         line.setShadowMode(RenderQueue.ShadowMode.Off);
         grid.attachChild(line);
      }

      GeometryBatchFactory.optimize(grid, true);
      zUpNode.attachChild(grid);
   }

   private void makeFloor()
   {
      Vector3f center = new Vector3f(0.0f, 0.0f, -0.25f);
      Box quad = new Box(0.0f, 0.0f, 0.0f);
      quad.updateGeometry(center, 100.0f, 100.0f, 0.25f);
      floor = new Node("floor Nodes");
      Node floorObject = new Node("floor object");
      Geometry floorGeometry = new Geometry("level-Floor", quad);
      floorGeometry.setLocalRotation(new Quaternion().fromAngleAxis(-FastMath.HALF_PI, Vector3f.UNIT_Z));

      Material mat = new Material(jme.getAssetManager(), "Common/MatDefs/Light/Lighting.j3md");
      mat.setBoolean("UseMaterialColors", true);

      mat.setColor("Diffuse", new ColorRGBA(1.0f, 1.0f, 1.0f, 0.0f));

      floorGeometry.setShadowMode(RenderQueue.ShadowMode.Receive);

      mat.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
      floorGeometry.setQueueBucket(RenderQueue.Bucket.Transparent);
      floorGeometry.setMaterial(mat);
      floorObject.attachChild(floorGeometry);
      floor.attachChild(floorObject);

//      attachFloor(floor);
   }

   public static void main(String[] args)
   {
      new JMEInSwingWindowEnvironment();
   }
}
