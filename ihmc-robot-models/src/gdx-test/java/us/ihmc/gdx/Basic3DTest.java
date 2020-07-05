package us.ihmc.gdx;

import com.badlogic.gdx.Application;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Application;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3ApplicationConfiguration;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.DirectionalLightsAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalLight;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.graphics.profiling.GLProfiler;
import com.badlogic.gdx.utils.viewport.ExtendViewport;
import com.badlogic.gdx.utils.viewport.Viewport;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple3D.Point3D;

import static com.badlogic.gdx.graphics.VertexAttributes.Usage;

/**
 * TODO: Pause and resume?
 */
public class Basic3DTest extends Lwjgl3ApplicationAdapter
{
   private static final int INITIAL_WIDTH = 1100;
   private static final int INITIAL_HEIGHT = 800;
   private FocusBasedGDXCamera camera;
   private ModelBatch modelBatch;
   private Environment environment;
   private Viewport viewport;
   private Model rootModel;
   private ModelInstance rootModelInstance;

   @Override
   public void create()
   {
      new GLProfiler(Gdx.graphics).enable();
      Gdx.app.setLogLevel(Application.LOG_INFO);

      environment = new Environment();
      environment.set(new ColorAttribute(ColorAttribute.AmbientLight, 0.4f, 0.4f, 0.4f, 1f));
      DirectionalLightsAttribute directionalLights = new DirectionalLightsAttribute();
      directionalLights.lights.add(new DirectionalLight().set(0.8f, 0.8f, 0.8f, -1f, -0.8f, -0.2f));
      environment.set(directionalLights);

      modelBatch = new ModelBatch();

      camera = new FocusBasedGDXCamera();
      viewport = new ExtendViewport(INITIAL_WIDTH, INITIAL_HEIGHT, camera);

      rootModel = new Model();

//      rootModel.nodes.addAll(camera.getFocusPointSphere().nodes);

      float distance = 5.0f;
      createBox(distance, distance, distance, Color.GREEN);
      createBox(-distance, distance, distance, Color.DARK_GRAY);
      createBox(distance, -distance, distance, Color.RED);
      createBox(-distance, -distance, distance, Color.ORANGE);
      createBox(distance, distance, -distance, Color.BLUE);
      createBox(-distance, distance, -distance, Color.BLACK);
      createBox(distance, -distance, -distance, Color.WHITE);
      createBox(-distance, -distance, -distance, Color.YELLOW);

      rootModel.nodes.addAll(createCoordinateFrame(0.3).nodes);

      rootModelInstance = new ModelInstance(rootModel);
      rootModelInstance.nodes.addAll(camera.getFocusPointSphere().nodes);
   }

   private void createBox(float x, float y, float z, Color color)
   {
      ModelBuilder modelBuilder = new ModelBuilder();
      Model boxDescription = modelBuilder.createBox(1f, 1f, 1f, new Material(ColorAttribute.createDiffuse(color)), Usage.Position | Usage.Normal);
      boxDescription.nodes.get(0).translation.set(x, y, z);
      boxDescription.calculateTransforms();
      rootModel.nodes.addAll(boxDescription.nodes);
   }

   public Model createCoordinateFrame(double length)
   {
      ModelBuilder modelBuilder = new ModelBuilder();
      modelBuilder.begin();
      modelBuilder.node().id = "coordinateFrame"; // optional

      double radius = 0.02 * length;
      double coneHeight = 0.10 * length;
      double coneRadius = 0.05 * length;
      GDXMultiColorMeshBuilder meshBuilder = new GDXMultiColorMeshBuilder();
      meshBuilder.addCylinder(length, radius, new Point3D(), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), Color.RED);
      meshBuilder.addCone(coneHeight, coneRadius, new Point3D(length, 0.0, 0.0), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), Color.RED);
      meshBuilder.addCylinder(length, radius, new Point3D(), new AxisAngle(1.0, 0.0, 0.0, -Math.PI / 2.0), Color.GREEN);
      meshBuilder.addCone(coneHeight, coneRadius, new Point3D(0.0, length, 0.0), new AxisAngle(1.0, 0.0, 0.0, -Math.PI / 2.0), Color.GREEN);
      meshBuilder.addCylinder(length, radius, new Point3D(), new AxisAngle(), Color.BLUE);
      meshBuilder.addCone(coneHeight, coneRadius, new Point3D(0.0, 0.0, length), new AxisAngle(), Color.BLUE);
      Mesh mesh = meshBuilder.generateMesh();

      MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL20.GL_TRIANGLES);
      Material material = new Material();
      Texture paletteTexture = new Texture(Gdx.files.classpath("palette.png"));
      material.set(TextureAttribute.createDiffuse(paletteTexture));
      material.set(ColorAttribute.createDiffuse(Color.WHITE));
      modelBuilder.part(meshPart, material);

      return modelBuilder.end();
   }

   @Override
   public void resize(int width, int height)
   {
      viewport.update(width, height);
   }

   @Override
   public void render()
   {
      Gdx.gl.glClearColor(0.5019608f, 0.5019608f, 0.5019608f, 1.0f);
      Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT | GL20.GL_DEPTH_BUFFER_BIT);

      camera.render();

      modelBatch.begin(camera);
      modelBatch.render(rootModelInstance, environment);
      // TODO add more render calls here
      modelBatch.end();
   }

   @Override
   public void dispose()
   {
      modelBatch.dispose();
      rootModel.dispose();
   }

   @Override
   public boolean closeRequested()
   {
      return true;
   }

   public static void main(String[] args)
   {
      Lwjgl3ApplicationConfiguration applicationConfiguration = new Lwjgl3ApplicationConfiguration();
      applicationConfiguration.setTitle("Basic3DTest");
      applicationConfiguration.setWindowedMode(INITIAL_WIDTH, INITIAL_HEIGHT);
      applicationConfiguration.useVsync(true);

      new Lwjgl3Application(new Basic3DTest(), applicationConfiguration);
   }
}