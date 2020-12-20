package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.InputMultiplexer;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Application;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3ApplicationConfiguration;
import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.DirectionalLightsAttribute;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalLight;
import com.badlogic.gdx.graphics.profiling.GLProfiler;
import com.badlogic.gdx.scenes.scene2d.Stage;
import com.badlogic.gdx.scenes.scene2d.ui.Skin;
import com.badlogic.gdx.scenes.scene2d.ui.Table;
import com.badlogic.gdx.scenes.scene2d.ui.TextButton;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import com.badlogic.gdx.utils.viewport.Viewport;

/**
 * TODO: Pause and resume?
 */
public class GDX3DDemo extends Lwjgl3ApplicationAdapter
{
   private static final int INITIAL_WIDTH = 1100;
   private static final int INITIAL_HEIGHT = 800;
   private FocusBasedGDXCamera camera3D;
   private Environment environment;
   private Viewport viewport;
   private ModelBatch modelBatch;
   private ModelInstance boxes;
   private ModelInstance coordinateFrame;

   private int currentWindowWidth;
   private int currentWindowHeight;
   private Stage stage;
   private Table table;

   public GDX3DDemo()
   {
      Lwjgl3ApplicationConfiguration applicationConfiguration = new Lwjgl3ApplicationConfiguration();
      applicationConfiguration.setTitle("GDX3DDemo");
      applicationConfiguration.setWindowedMode(INITIAL_WIDTH, INITIAL_HEIGHT);
      applicationConfiguration.useVsync(true);

      new Lwjgl3Application(this, applicationConfiguration);
   }

   @Override
   public void create()
   {
      new GLProfiler(Gdx.graphics).enable();
      GDXTools.syncLogLevelWithLogTools();

      environment = new Environment();
      environment.set(new ColorAttribute(ColorAttribute.AmbientLight, 0.4f, 0.4f, 0.4f, 1f));
      DirectionalLightsAttribute directionalLights = new DirectionalLightsAttribute();
      directionalLights.lights.add(new DirectionalLight().set(0.8f, 0.8f, 0.8f, -1f, -0.8f, -0.2f));
      environment.set(directionalLights);

      modelBatch = new ModelBatch();

      InputMultiplexer inputMultiplexer = new InputMultiplexer();
      Gdx.input.setInputProcessor(inputMultiplexer);

      camera3D = new FocusBasedGDXCamera();
      inputMultiplexer.addProcessor(camera3D.getInputProcessor());
      viewport = new ScreenViewport(camera3D);

      coordinateFrame = new ModelInstance(GDXModelPrimitives.createCoordinateFrame(0.3));
      boxes = new BoxesDemoModel().newInstance();

      stage = new Stage(new ScreenViewport());
      inputMultiplexer.addProcessor(stage);

      table = new Table();
      table.setFillParent(true);
//      table.setDebug(true);
      stage.addActor(table);

      Skin skin = new Skin(Gdx.files.internal("uiskin.json"));

      table.left();
      table.top();
      TextButton button1 = new TextButton("Button 1", skin);
      table.add(button1);

      TextButton button2 = new TextButton("Button 2", skin);
      table.add(button2);
   }

   @Override
   public void resize(int width, int height)
   {
      this.currentWindowWidth = width;
      this.currentWindowHeight = height;
   }

   @Override
   public void render()
   {
      Gdx.gl.glClearColor(0.5019608f, 0.5019608f, 0.5019608f, 1.0f);
      Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT | GL20.GL_DEPTH_BUFFER_BIT);
      Gdx.gl.glEnable(GL20.GL_TEXTURE_2D);

      viewport.update(currentWindowWidth, currentWindowHeight * 3 / 4);
      Gdx.gl.glViewport(0, Gdx.graphics.getHeight()* 1 / 4, Gdx.graphics.getWidth(), Gdx.graphics.getHeight()* 3 / 4);

      camera3D.render();

      modelBatch.begin(camera3D);
      modelBatch.render(camera3D.getFocusPointSphere(), environment);
      modelBatch.render(boxes, environment);
      modelBatch.render(coordinateFrame, environment);
      modelBatch.end();

      Gdx.gl.glViewport(0, 0, Gdx.graphics.getWidth(), Gdx.graphics.getHeight()* 1 / 4);
      stage.getViewport().update(currentWindowWidth, currentWindowHeight * 1 / 4, true);

      stage.act(Gdx.graphics.getDeltaTime());
      stage.draw();
   }

   @Override
   public void dispose()
   {
      boxes.model.dispose();
      coordinateFrame.model.dispose();
      camera3D.dispose();
      modelBatch.dispose();
      stage.dispose();
   }

   @Override
   public boolean closeRequested()
   {
      return true;
   }

   public static void main(String[] args)
   {
      new GDX3DDemo();
   }
}