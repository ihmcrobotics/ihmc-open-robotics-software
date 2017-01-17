package us.ihmc.jMonkeyEngineToolkit.holders;

import us.ihmc.jMonkeyEngineToolkit.camera.CameraController;
import us.ihmc.jMonkeyEngineToolkit.camera.ViewportAdapter;

public class ActiveViewportHolder {
	private static ActiveViewportHolder instance = new ActiveViewportHolder();

	public static ActiveViewportHolder getInstance() {
		return instance;
	}

	private ViewportAdapter activeViewPort = null;

	private ActiveViewportHolder() {

	}

	public ViewportAdapter getActiveViewport() {
		return activeViewPort;
	}

	public void setActiveViewport(ViewportAdapter viewport) {
		this.activeViewPort = viewport;
	}

	public boolean isActiveViewport(ViewportAdapter viewport) {
		return this.activeViewPort == viewport;
	}

	public boolean isActiveCamera(CameraController cameraController) {
		return this.activeViewPort.getCameraController() == cameraController;
	}

}
