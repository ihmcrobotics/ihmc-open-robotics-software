package us.ihmc.robotbuilder.util;

import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;
import javafx.application.Platform;
import javafx.geometry.Bounds;
import javafx.scene.Node;
import javafx.scene.PerspectiveCamera;
import javafx.scene.transform.Affine;
import javafx.scene.transform.MatrixType;
import javafx.scene.transform.Rotate;
import javaslang.concurrent.Future;
import javaslang.concurrent.Promise;

import java.util.concurrent.Callable;

/**
 * Utility methods
 */
public class Util {
    /**
     * Wraps {@link Platform#runLater(Runnable)} to use a {@link Future} which allows chaining
     * and waiting for actions.
     * @param callable code to run in the UI thread
     * @param <T> return type of the UI code
     * @return {@link Future} instance for the given code that is fulfilled as soon as the given code executes in the UI thread
     */
    public static <T> Future<T> runLaterInUI(Callable<? extends T> callable) {
        Promise<T> result = Promise.make();
        Platform.runLater(() -> {
            try {
                result.success(callable.call());
            } catch (Throwable thr) {
                result.failure(thr);
            }
        });
        return result.future();
    }

    /**
     * Computes camera settings so that the camera shows the whole node (based on its bounding box) from the direction
     * given by the dir vector if corner == false or from the direction of a bounding box corner if corner == true
     * @param dir direction where to look at the node from (if corner == false) or the corner (if corner == true)
     *            in which case dir identifies corners as if they were on a [0;1]^3 cube
     * @param node node to look at, root node is used if the given node is null
     * @param width camera resolution width
     * @param height camera resolution height
     * @param up up vector
     */
    public static PerspectiveCamera lookAtNodeFromDirection(final int width, final int height, final Vector3f dir, final Node node, final Vector3f up) {
        Camera tmpCamera = new Camera(width, height);

        // We get a bounding sphere of the node and calculate how far the camera needs
        // to be to fit in the whole sphere in its view.


        final Bounds box = node.getBoundsInParent();
        if (box.isEmpty()) {
            return new PerspectiveCamera();
        }

        Vector3f dirVec = dir;
        Vector3f min = new Vector3f((float)box.getMinX(), (float)box.getMinY(), (float)box.getMinZ());
        Vector3f max = new Vector3f((float)box.getMaxX(), (float)box.getMaxY(), (float)box.getMaxZ());
        Vector3f center = min.add(max.mult(0.5f));

        dirVec = dirVec.normalize();

        float r = center.subtract(min).length();
        float h = tmpCamera.getFrustumTop();
        float near = tmpCamera.getFrustumNear();
        float d = r * near / h;

        float farR = 2 * r;

        Vector3f loc = dirVec.mult(-d).add(center);
        tmpCamera.setLocation(loc);
        tmpCamera.lookAt(center, up);
        tmpCamera.setFrustumFar(Math.max(10 * (farR + d), tmpCamera.getFrustumFar()));

        PerspectiveCamera result = new PerspectiveCamera(true);
        float fov = (float)Math.abs(Math.atan(tmpCamera.getFrustumTop() / tmpCamera.getFrustumNear())) * 2 * FastMath.RAD_TO_DEG;
        result.setFieldOfView(fov);
        result.setNearClip(tmpCamera.getFrustumNear());
        result.setFarClip(tmpCamera.getFrustumFar());
        result.getTransforms().add(new Rotate(180, Rotate.Z_AXIS)); // y up
        float[] viewMat = new float[16];
        tmpCamera.getViewMatrix().fillFloatArray(viewMat, false);
        double[] viewMatDouble = new double[16];
        for (int i = 0; i < viewMat.length; i++) {
            viewMatDouble[i] = viewMat[i];
        }

        result.getTransforms().add(new Affine(viewMatDouble, MatrixType.MT_3D_4x4, 0));
        return result;
    }
}
