package us.ihmc.rdx.ui;

import com.badlogic.gdx.math.Vector3;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

/**
 * Frustum collision check between near and far planes.
 * Works for convex quad frustums aligned with a camera.
 */
public class CameraPanelFrustumCollision
{
    // Working (world-space) coordinates used in collision checks
    private final Vector3[] nearCorners = new Vector3[4];
    private final Vector3[] farCorners  = new Vector3[4];

    // Original local-space reference (for repeated transforms without drift)
    private final Vector3[] localNearCorners = new Vector3[4];
    private final Vector3[] localFarCorners  = new Vector3[4];

    public CameraPanelFrustumCollision(Vector3[] nearCorners, Vector3[] farCorners)
    {
        for (int i = 0; i < 4; i++)
        {
            // Store working copies
            this.nearCorners[i] = new Vector3(nearCorners[i]);
            this.farCorners[i]  = new Vector3(farCorners[i]);

            // Store originals (local space) for future transforms
            this.localNearCorners[i] = new Vector3(nearCorners[i]);
            this.localFarCorners[i]  = new Vector3(farCorners[i]);
        }
    }

    public void updateCorners(RigidBodyTransform transformToApply)
    {
        Point3D temp = new Point3D();
        for (int i = 0; i < 4; i++)
        {
            // Transform from the *original* (local) coordinates
            temp.set(localNearCorners[i].x, localNearCorners[i].y, localNearCorners[i].z);
            transformToApply.transform(temp);
            nearCorners[i].set((float) temp.getX(), (float) temp.getY(), (float) temp.getZ());

            temp.set(localFarCorners[i].x, localFarCorners[i].y, localFarCorners[i].z);
            transformToApply.transform(temp);
            farCorners[i].set((float) temp.getX(), (float) temp.getY(), (float) temp.getZ());
        }
    }

    /**
     * Checks if a point is inside the frustum.
     * @param point world-space coordinate to test.
     */
    public boolean isPointInside(Vector3 point)
    {
        return isInsideFace(nearCorners[3], farCorners[3], farCorners[2], point) && // Right
               isInsideFace(nearCorners[0], farCorners[0], farCorners[3], point) && // Top
               isInsideFace(nearCorners[1], farCorners[1], farCorners[0], point) && // Left
               isInsideFace(nearCorners[2], farCorners[2], farCorners[1], point) && // Bottom
               isInsideFace(farCorners[0], farCorners[1], farCorners[2], point);    // Far plane
    }

    private boolean isInsideFace(Vector3 a, Vector3 b, Vector3 c, Vector3 p)
    {
        // Same-side test using cross product
        Vector3 ab = new Vector3(b).sub(a);
        Vector3 ac = new Vector3(c).sub(a);
        Vector3 ap = new Vector3(p).sub(a);

        Vector3 normal = ab.crs(ac).nor();
        float dist = normal.dot(ap);
        // For inside test, point must be on the same side as frustum interior
        return dist >= 0.0001f;
    }
}