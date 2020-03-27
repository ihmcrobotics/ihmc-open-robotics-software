package us.ihmc.atlas.behaviors.libgdx;

import com.badlogic.gdx.Application;
import com.badlogic.gdx.ApplicationListener;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Application;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3ApplicationConfiguration;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.PerspectiveCamera;
import com.badlogic.gdx.graphics.VertexAttributes;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalLight;
import com.badlogic.gdx.graphics.g3d.utils.CameraInputController;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;

public class Basic3DTest implements ApplicationListener {
   public PerspectiveCamera cam;
   public ModelBatch modelBatch;
   public Model model;
   public ModelInstance instance;
   private CameraInputController camController;
   private Environment environment;
   private Renderable renderable;

   @Override
   public void create() {

      environment = new Environment();
      environment.set(new ColorAttribute(ColorAttribute.AmbientLight, 0.4f, 0.4f, 0.4f, 1f));
      environment.add(new DirectionalLight().set(0.8f, 0.8f, 0.8f, -1f, -0.8f, -0.2f));

      modelBatch = new ModelBatch();

      cam = new PerspectiveCamera(67, Gdx.graphics.getWidth(), Gdx.graphics.getHeight());
      cam.position.set(10f, 10f, 10f);
      cam.lookAt(0,0,0);
      cam.near = 1f;
      cam.far = 300f;
      cam.update();

      camController = new CameraInputController(cam);
      Gdx.input.setInputProcessor(camController);

      ModelBuilder modelBuilder = new ModelBuilder();
      model = modelBuilder.createBox(5f, 5f, 5f,
                                     new Material(ColorAttribute.createDiffuse(Color.GREEN)),
                                     VertexAttributes.Usage.Position | VertexAttributes.Usage.Normal);
      instance = new ModelInstance(model);

      renderable = new Renderable();
//      instance.setRenderable(renderable);
      renderable.environment = environment;
      renderable.worldTransform.idt();
   }

   @Override
   public void render() {
      Gdx.gl.glViewport(0, 0, Gdx.graphics.getWidth(), Gdx.graphics.getHeight());
      Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT | GL20.GL_DEPTH_BUFFER_BIT);

      modelBatch.begin(cam);
      modelBatch.render(instance, environment);
      modelBatch.end();
   }

   @Override
   public void dispose() {
      modelBatch.dispose();
      model.dispose();
   }

   @Override
   public void resize(int width, int height) {
   }

   @Override
   public void pause() {
   }

   @Override
   public void resume() {
   }

   public static void main(String[] args)
   {
      Lwjgl3ApplicationConfiguration cfg = new Lwjgl3ApplicationConfiguration();
      cfg.setTitle("Basic3DTest");
      cfg.setWindowedMode(1100, 800);
      cfg.useVsync(true);

      new Lwjgl3Application(new Basic3DTest(), cfg);
   }
}
