package us.ihmc.avatar.mocapRetarget.bvh.SkeletonHierarchy;

import us.ihmc.euclid.tuple3D.Vector3D;

import java.util.List;

public record JointInfo(String name, String parentName, Vector3D offset, List<String> channels)
{
}
