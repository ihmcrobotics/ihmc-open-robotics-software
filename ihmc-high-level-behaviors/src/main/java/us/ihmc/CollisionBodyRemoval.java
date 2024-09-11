//package us.ihmc;
///*
// * This class handles the creation of a height map while ignoring specific parts of the robot.
// *
// * - The class takes in a robot collision model.
// * - It takes in a point cloud as input.
// * - It checks if any point in the point cloud intersects with the robot's collision model.
// * - If a point intersects with the collision model, that point is removed from the point cloud.
// * - After the removal process, the height map is created using the filtered point cloud.
// *
// * Note:
// * - This class was designed to exclude the robot's knees and feet when generating the height map.
// */
//
//public class CollisionBodyRemoval
//{
//   private RobotCollisionModel robotCollisionModel;
//   private List<Point3D> pointCloud; // A 3D point cloud
//   private List<Point3D> filteredPointCloud;
//
//   public CollisionBodyRemoval(RobotCollisionModel robotCollisionModel, List<Point3D> pointCloud)
//   {
//      this.robotCollisionModel = robotCollisionModel;
//      this.pointCloud = pointCloud;
//      this.filteredPointCloud = new ArrayList<>();
//   }
//
//   public void filterPointCloud()
//   {
//      //check if point is intersecting with the collision model, then delete it from it
//   }
//}