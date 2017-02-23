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

import us.ihmc.euclid.tuple3D.Vector3D;
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
    public static PerspectiveCamera lookAtNodeFromDirection(final Node node, double fovDegrees, final Vector3D dir, final Vector3D up) {
        // We get a bounding sphere of the node and calculate how far the camera needs
        // to be to fit in the whole sphere in its view.
        final Bounds box = node.getBoundsInParent();
        if (box.isEmpty()) {
            return new PerspectiveCamera(true);
        }

        Vector3D dirVec = new Vector3D(dir);
        dirVec.normalize();
        Vector3D bboxMin = new Vector3D(box.getMinX(), box.getMinY(), box.getMinZ());
        Vector3D bboxMax = new Vector3D(box.getMaxX(), box.getMaxY(), box.getMaxZ());
        Vector3D sceneCenter = new Vector3D(bboxMin);
        sceneCenter.add(bboxMax);
        sceneCenter.scale(0.5f);

        Vector3D radiusVec = new Vector3D(sceneCenter);
        radiusVec.sub(bboxMin);
        double r = radiusVec.length(); // scene bounding sphere radius
        double near = r / 1000; // camera near, giving some space for zoom-in
        double h = Math.sin(Math.toRadians(fovDegrees) / 2) * near; // half of the height of the projection plane
        double d = r * near / h; // how far we need to be for the bounding sphere to fit in the projection rectangle

        double far = 10 * (2 * r + d); // (distance from the scene + scene bounding sphere diameter) * 10 (allows a 10x zoom-out)

        Vector3D cameraLocation = new Vector3D(dirVec);
        cameraLocation.scale(-d);
        cameraLocation.add(sceneCenter);

        PerspectiveCamera result = new PerspectiveCamera(true);
        result.setFieldOfView(fovDegrees);
        result.setNearClip(near);
        result.setFarClip(far);

        result.getTransforms().add(new Translate(cameraLocation.getX(), cameraLocation.getY(), cameraLocation.getZ()));
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
    private static Affine lookAt(Vector3D cameraLocation, Vector3D target, Vector3D up) {
        // Adjusted from JMonkey Engine code
        Vector3D newDirection = new Vector3D();
        Vector3D newUp = new Vector3D();
        Vector3D newLeft = new Vector3D();

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
            if (newDirection.getX() != 0) {
                //noinspection SuspiciousNameCombination
                newLeft.set(newDirection.getY(), -newDirection.getX(), 0f);
            } else {
                newLeft.set(0f, newDirection.getZ(), -newDirection.getY());
            }
        }

        newUp.set(newDirection);
        newUp.cross(newDirection, newLeft);
        newUp.normalize();

        return new Affine(new double[] {
            newLeft.getX(), newLeft.getY(), newLeft.getZ(), 0,
            newUp.getX(), newUp.getY(), newUp.getZ(), 0,
            newDirection.getX(), newDirection.getY(), newDirection.getZ(), 0,
            0, 0, 0, 1
        }, MatrixType.MT_3D_4x4, 0);
    }
}
