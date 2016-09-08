package us.ihmc.robotbuilder.util;

import javafx.application.Platform;
import javafx.geometry.Bounds;
import javafx.scene.Node;
import javafx.scene.PerspectiveCamera;
import javafx.scene.transform.Affine;
import javafx.scene.transform.MatrixType;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import javaslang.concurrent.Future;
import javaslang.concurrent.Promise;

import javax.vecmath.Vector3d;
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
     * @param node node to look at
     * @param fovDegrees camera field of view in degrees
     * @param dir direction where to look at the node from
     * @param up up vector
     */
    public static PerspectiveCamera lookAtNodeFromDirection(final Node node, double fovDegrees, final Vector3d dir, final Vector3d up) {
        // We get a bounding sphere of the node and calculate how far the camera needs
        // to be to fit in the whole sphere in its view.
        final Bounds box = node.getBoundsInParent();
        if (box.isEmpty()) {
            return new PerspectiveCamera(true);
        }

        Vector3d dirVec = new Vector3d(dir);
        dirVec.normalize();
        Vector3d bboxMin = new Vector3d(box.getMinX(), box.getMinY(), box.getMinZ());
        Vector3d bboxMax = new Vector3d(box.getMaxX(), box.getMaxY(), box.getMaxZ());
        Vector3d sceneCenter = new Vector3d(bboxMin);
        sceneCenter.add(bboxMax);
        sceneCenter.scale(0.5f);

        Vector3d radiusVec = new Vector3d(sceneCenter);
        radiusVec.sub(bboxMin);
        double r = radiusVec.length(); // scene bounding sphere radius
        double near = r / 1000; // camera near, giving some space for zoom-in
        double h = Math.sin(Math.toRadians(fovDegrees) / 2) * near; // half of the height of the projection plane
        double d = r * near / h; // how far we need to be for the bounding sphere to fit in the projection rectangle

        double far = 10 * (2 * r + d); // (distance from the scene + scene bounding sphere diameter) * 10 (allows a 10x zoom-out)

        Vector3d cameraLocation = new Vector3d(dirVec);
        cameraLocation.scale(-d);
        cameraLocation.add(sceneCenter);

        PerspectiveCamera result = new PerspectiveCamera(true);
        result.setFieldOfView(fovDegrees);
        result.setNearClip(near);
        result.setFarClip(far);

        result.getTransforms().add(new Translate(cameraLocation.x, cameraLocation.y, cameraLocation.z));
        result.getTransforms().add(lookAt(cameraLocation, sceneCenter, up));
        // Adjust to JavaFX world (z up, right handed)
        result.getTransforms().add(new Rotate(180, Rotate.X_AXIS));
        result.getTransforms().add(new Rotate(90, Rotate.Z_AXIS));
        return result;
    }

    /**
     * Get the rotation matrix from a camera location, target location and camera up vector.
     * Returns a 4x4 affine matrix for convenience.
     * @param cameraLocation camera location
     * @param target target location
     * @param up up vector
     * @return transform representing the rotation
     */
    private static Affine lookAt(Vector3d cameraLocation, Vector3d target, Vector3d up) {
        // Adjusted from JMonkey Engine code
        Vector3d newDirection = new Vector3d();
        Vector3d newUp = new Vector3d();
        Vector3d newLeft = new Vector3d();

        newDirection.set(target);
        newDirection.sub(cameraLocation);
        newDirection.normalize();

        newUp.set(up);
        newUp.normalize();
        if (newUp.lengthSquared() < 1e-8) {
            newUp.set(0, 1, 0);
        }

        newLeft.cross(newUp, newDirection);
        newLeft.normalize();
        if (newLeft.lengthSquared() < 1e-8) {
            if (newDirection.x != 0) {
                //noinspection SuspiciousNameCombination
                newLeft.set(newDirection.y, -newDirection.x, 0f);
            } else {
                newLeft.set(0f, newDirection.z, -newDirection.y);
            }
        }

        newUp.set(newDirection);
        newUp.cross(newDirection, newLeft);
        newUp.normalize();

        return new Affine(new double[] {
            newLeft.x, newLeft.y, newLeft.z, 0,
            newUp.x, newUp.y, newUp.z, 0,
            newDirection.x, newDirection.y, newDirection.z, 0,
            0, 0, 0, 1
        }, MatrixType.MT_3D_4x4, 0);
    }
}
