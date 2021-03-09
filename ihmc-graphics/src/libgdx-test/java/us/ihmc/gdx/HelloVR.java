package us.ihmc.gdx;

import com.badlogic.gdx.ApplicationAdapter;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Application;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3ApplicationConfiguration;
import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.FPSLogger;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.PerspectiveCamera;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Pixmap.Format;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.VertexAttributes.Usage;
import com.badlogic.gdx.graphics.g3d.Environment;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalLight;
import com.badlogic.gdx.graphics.g3d.loader.G3dModelLoader;
import com.badlogic.gdx.graphics.g3d.utils.FirstPersonCameraController;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.graphics.glutils.ShapeRenderer;
import com.badlogic.gdx.graphics.glutils.ShapeRenderer.ShapeType;
import com.badlogic.gdx.math.Intersector;
import com.badlogic.gdx.math.Plane;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.math.collision.Ray;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.JsonReader;

import org.lwjgl.opengl.GL32;
import us.ihmc.gdx.vr.GDXVRCamera;
import us.ihmc.gdx.vr.GDXVRContext;
import us.ihmc.gdx.vr.GDXVRContext.*;

public class HelloVR extends ApplicationAdapter {
	static final String TAG = "HelloVR";

	/**
	 * The {@link GDXVRContext} setup in createVR(), may be null if no HMD is
	 * present or SteamVR is not installed
	 */
	GDXVRContext context;

	/** Camera used to render to the desktop Window **/
	PerspectiveCamera companionCamera;
	/** {@link FirstPersonCameraController} used in case VR is not available **/
	FirstPersonCameraController cameraController;
	/** {@link ModelBatch} used to render **/
	ModelBatch batch;
	/** {@link ShapeRenderer} to draw lines for axes etc. **/
	ShapeRenderer renderer;
	/** {@link Environment} storing out light configuration **/
	Environment environment;
	/** A cube model shared by all cube instances **/
	Model cubeModel;
	/** A disc model used to display the teleportation target **/
	Model discModel;
	/** The corresponding {@link ModelInstance} used for rendering **/
	ModelInstance disc;
	/** All {@link ModelInstance}s to be rendered **/
	Array<ModelInstance> modelInstances = new Array<ModelInstance>();
	/** Wheter we are teleporting or not **/
	boolean isTeleporting = false;
	/** An {@link FPSLogger} for tracking the frame rate **/
	FPSLogger logger = new FPSLogger();

	@Override
	public void create() {
		createScene();
		createVR();
	}

	private void createScene() {
		companionCamera = new PerspectiveCamera(67, Gdx.graphics.getWidth(), Gdx.graphics.getHeight());
		companionCamera.near = 0.1f;
		companionCamera.far = 1000f;

		batch = new ModelBatch();
		renderer = new ShapeRenderer();

		ModelBuilder modelBuilder = new ModelBuilder();
		discModel = modelBuilder.createCylinder(1, 0.1f, 1, 20, new Material(ColorAttribute.createDiffuse(Color.CORAL)),
				Usage.Position | Usage.Normal);
		disc = new ModelInstance(discModel);

		Pixmap pixmap = new Pixmap(64, 64, Format.RGBA8888);
		pixmap.setColor(Color.WHITE);
		pixmap.fill();
		pixmap.setColor(Color.DARK_GRAY);
		pixmap.fillRectangle(0, 0, 32, 32);
		pixmap.fillRectangle(32, 32, 32, 32);
		Texture texture = new Texture(pixmap);

		cubeModel = modelBuilder.createBox(1f, 1f, 1f,
				new Material(ColorAttribute.createDiffuse(Color.GREEN), TextureAttribute.createDiffuse(texture)),
				Usage.Position | Usage.Normal | Usage.TextureCoordinates);

		Color[] colors = new Color[] { Color.RED, Color.GREEN, Color.BLUE, Color.YELLOW, Color.MAGENTA, Color.GOLD,
				Color.ORANGE };

		int idx = 0;
		for (int z = -3; z <= 3; z += 3) {
			for (float y = -0.5f; y <= 9 - 0.5f; y += 3) {
				for (int x = -3; x <= 3; x += 3) {
					if (x == 0 && y == 0 && z == 0)
						continue;
					ModelInstance cube = new ModelInstance(cubeModel);
					cube.materials.get(0).get(ColorAttribute.class, ColorAttribute.Diffuse).color.set(colors[idx++]);
					if (idx > colors.length - 1)
						idx = 0;
					cube.transform.translate(x, y, z);
					modelInstances.add(cube);
				}
			}
		}

		Model apartmentModel = new G3dModelLoader(new JsonReader()).loadModel(Gdx.files.internal("apartment.g3dj"));
		ModelInstance apartment = new ModelInstance(apartmentModel);
		apartment.transform.scale(1 / 100f, 1 / 100f, 1 / 100f);
		modelInstances.add(apartment);

		environment = new Environment();
		environment.set(new ColorAttribute(ColorAttribute.AmbientLight, 0.4f, 0.4f, 0.4f, 1f));
		environment.add(new DirectionalLight().set(0.8f, 0.8f, 0.8f, -1f, -0.8f, -0.2f));
	}

	private void createVR() {
		// Initializing the GDXVRContext may fail if no HMD is connected or SteamVR
		// is not installed.
		try {
			context = new GDXVRContext();

			// Set the far clip plane distance on the camera of each eye
			// All units are in meters.
			context.getEyeData(Eye.Left).camera.far = 100f;
			context.getEyeData(Eye.Right).camera.far = 100f;

			// Register a VRDeviceListener to get notified when
			// controllers are (dis-)connected and their buttons
			// are pressed. Note that we add/remove a ModelInstance for
			// controllers for rendering on (dis-)connect.
			context.addListener(new VRDeviceListener() {
				@Override
				public void connected(VRDevice device) {
					Gdx.app.log(TAG, device + " connected");
					if (device.getType() == VRDeviceType.Controller && device.getModelInstance() != null)
						modelInstances.add(device.getModelInstance());
				}

				@Override
				public void disconnected(VRDevice device) {
					Gdx.app.log(TAG, device + " disconnected");
					if (device.getType() == VRDeviceType.Controller && device.getModelInstance() != null)
						modelInstances.removeValue(device.getModelInstance(), true);
				}

				@Override
				public void buttonPressed(VRDevice device, int button) {
					Gdx.app.log(TAG, device + " button pressed: " + button);

					// If the trigger button on the first controller was
					// pressed, setup teleporting
					// mode.
					if (device == context.getDeviceByType(VRDeviceType.Controller)) {
						if (button == VRControllerButtons.SteamVR_Trigger)
							isTeleporting = true;
					}
				}

				@Override
				public void buttonReleased(VRDevice device, int button) {
					Gdx.app.log(TAG, device + " button released: " + button);

					// If the trigger button the first controller was released,
					// teleport the player.
					if (device == context.getDeviceByType(VRDeviceType.Controller)) {
						if (button == VRControllerButtons.SteamVR_Trigger) {
							if (intersectControllerXZPlane(context.getDeviceByType(VRDeviceType.Controller), tmp)) {
								// Teleportation
								// - Tracker space origin in world space is initially at [0,0,0]
								// - When teleporting, we want to set the tracker space origin in world space to the
								//   teleportation point
								// - Then we need to offset the tracker space
								//   origin in world space by the camera
								//   x/z position so the camera is at the
								//   teleportation point in world space
								tmp2.set(context.getDeviceByType(VRDeviceType.HeadMountedDisplay).getPosition(Space.Tracker));
								tmp2.y = 0;
								tmp.sub(tmp2);

								context.getTrackerSpaceOriginToWorldSpaceTranslationOffset().set(tmp);
							}
							isTeleporting = false;
						}
					}
				}
			});
		} catch (Exception e) {
			// If initializing the GDXVRContext failed, we fall back
			// to desktop only mode with a FirstPersonCameraController.
			cameraController = new FirstPersonCameraController(companionCamera);
			Gdx.input.setInputProcessor(cameraController);

			// Set the camera height to 1.7m to emulate an
			// average human's height. We'd get this from the
			// HMD tracking otherwise.
			companionCamera.position.y = 1.7f;

			// We also enable vsync which the GDXVRContext would have
			// managed otherwise
			Gdx.graphics.setVSync(true);
		}
	}

	Plane xzPlane = new Plane(Vector3.Y, 0);
	Ray ray = new Ray();
	Vector3 tmp = new Vector3();
	Vector3 tmp2 = new Vector3();

	private boolean intersectControllerXZPlane(VRDevice controller, Vector3 intersection) {
		ray.origin.set(controller.getPosition(Space.World));
		ray.direction.set(controller.getDirection(Space.World).nor());
		return Intersector.intersectRayPlane(ray, xzPlane, intersection);
	}

	@Override
	public void render() {
		logger.log();

		// In case initializing the GDXVRContext succeeded
		if (context != null) {
			// poll the latest tracking data. must be called
			// before context.begin()!
			context.pollEvents();

			// check if we are teleporting (first controller trigger
			// button is down)
			modelInstances.removeValue(disc, true);
			if (isTeleporting) {
				// Intersect a ray along the controller's pointing direction
				// with the
				// xz plane. If there's an intersection, place the disc at the
				// position
				// and add it to the model instances to be rendered.
				if (intersectControllerXZPlane(context.getDeviceByType(VRDeviceType.Controller), tmp)) {
					disc.transform.idt().translate(tmp);
					modelInstances.add(disc);
				}
			}

			// render the scene for the left/right eye
			context.begin();
			renderScene(Eye.Left);
			renderScene(Eye.Right);
			context.end();

			// Render to the companion window (manually, see GDXVRContext for
			// helpers)
			VRDevice hmd = context.getDeviceByType(VRDeviceType.HeadMountedDisplay);
			companionCamera.direction.set(hmd.getDirection(Space.World));
			companionCamera.up.set(hmd.getUp(Space.World));
			companionCamera.position.set(hmd.getPosition(Space.World));
			companionCamera.update();
			renderScene(companionCamera);
		} else {
			// In desktop only mode, we just update the camera
			// controller
			cameraController.update();
			renderScene(companionCamera);
		}
	}

	Vector3 position = new Vector3();
	Vector3 xAxis = new Vector3();
	Vector3 yAxis = new Vector3();
	Vector3 zAxis = new Vector3();

	private void renderScene(Eye eye) {
		GDXVRCamera camera = context.getEyeData(eye).camera;
		context.beginEye(eye);
		renderScene(camera);
		context.endEye();
	}

	private void renderScene(Camera camera) {
		Gdx.gl.glClearColor(0.2f, 0.2f, 0.2f, 1);
		Gdx.gl.glClear(GL32.GL_COLOR_BUFFER_BIT | GL32.GL_DEPTH_BUFFER_BIT);

		// render all the models in the scene
		batch.begin(camera);
		for (ModelInstance modelInstance : modelInstances)
			batch.render(modelInstance, environment);
		batch.end();

		// render coordinate system axes for orientation
		renderer.setProjectionMatrix(camera.combined);
		renderer.begin(ShapeType.Line);
		renderer.setColor(Color.WHITE);
		renderer.line(-100, 0, 0, 0, 0, 0);
		renderer.line(0, -100, 0, 0, 0, 0);
		renderer.line(0, 0, -100, 0, 0, 0);
		renderer.setColor(Color.RED);
		renderer.line(0, 0, 0, 100, 0, 0);
		renderer.setColor(Color.GREEN);
		renderer.line(0, 0, 0, 0, 100, 0);
		renderer.setColor(Color.BLUE);
		renderer.line(0, 0, 0, 0, 0, 100);
		renderer.end();

		// render direction, up and right axes of each controller if
		// the GDXVRContext was successfully created
		if (context != null) {
			renderer.begin(ShapeType.Line);
			for (VRDevice device : context.getDevices()) {
				if (device.getType() == VRDeviceType.Controller) {
					renderer.setColor(Color.BLUE);
					Vector3 pos = tmp.set(device.getPosition(Space.World));
					Vector3 dir = tmp2.set(device.getDirection(Space.World)).scl(0.5f);
					renderer.line(device.getPosition(Space.World), pos.add(dir));

					renderer.setColor(Color.GREEN);
					pos = tmp.set(device.getPosition(Space.World));
					dir = tmp2.set(device.getUp(Space.World)).scl(0.1f);
					renderer.line(device.getPosition(Space.World), pos.add(dir));

					renderer.setColor(Color.RED);
					pos = tmp.set(device.getPosition(Space.World));
					dir = tmp2.set(device.getRight(Space.World)).scl(0.1f);
					renderer.line(device.getPosition(Space.World), pos.add(dir));
				}
			}
			renderer.end();
		}
	}

	@Override
	public void dispose() {
		if (context != null)
			context.dispose();
		batch.dispose();
		renderer.dispose();
		cubeModel.dispose();
		discModel.dispose();
	}

	public static void main(String[] args) {
		Lwjgl3ApplicationConfiguration config = new Lwjgl3ApplicationConfiguration();
		// Note that we disable VSync! The GDXVRContext manages vsync with respect
		// to
		// the HMD
		config.useVsync(false);
		config.setWindowedMode(800, 600);
		new Lwjgl3Application(new HelloVR(), config);
	}
}