#include "FactorGraphHandler.h"



FactorGraphHandler::FactorGraphHandler()
{
   /* Set ISAM2 parameters here. */
   parameters.relinearizeThreshold = 0.01;
   parameters.relinearizeSkip = 1;
   this->isam = gtsam::ISAM2(parameters);

   // gtsam::Vector6 odomVariance;
   // odomVariance << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2;
   // createOdometryNoiseModel(odomVariance);

   // gtsam::Vector3 lmVariance;
   // lmVariance << 1e-2, 1e-2, 1e-2;
   // createOrientedPlaneNoiseModel(lmVariance);

   gtsam::Vector6 priorVariance;
   priorVariance << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
   priorNoise = gtsam::noiseModel::Diagonal::Variances(priorVariance);

   gtsam::Vector6 priorVariance2;
   priorVariance2 << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2;
   priorNoise2 = gtsam::noiseModel::Diagonal::Variances(priorVariance2);

   pointLandmarkNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);

   /* TODO: Create setter for camera params.*/
   // KITTI Left: 718.856, 718.856, 607.193, 185.216
   K = gtsam::Cal3_S2::shared_ptr(new gtsam::Cal3_S2(718.856, 718.856, 0.0, 607.193, 185.216));
}

void FactorGraphHandler::createOdometryNoiseModel(gtsam::Vector6 odomVariance)
{
   odometryNoise = gtsam::noiseModel::Diagonal::Variances(odomVariance);
}

void FactorGraphHandler::createOrientedPlaneNoiseModel(gtsam::Vector3 lmVariances)
{
   orientedPlaneNoise = gtsam::noiseModel::Diagonal::Variances(lmVariances);
}

void FactorGraphHandler::addPriorPoseFactor(int index, gtsam::Pose3 mean)
{
   // printf("Prior Pose Factor: x%d\n", index); fflush(stdout);
   graph.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', index), mean, priorNoise));
}

void FactorGraphHandler::addOdometryFactor(gtsam::Pose3 odometry, int poseId)
{
   // printf("Odometry Factor: x%d -> x%d\n", poseId - 1, poseId); fflush(stdout);
   graph.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol('x', poseId - 1), gtsam::Symbol('x', poseId), odometry, odometryNoise));
   poseId++;
}

void FactorGraphHandler::addOrientedPlaneFactor(gtsam::Vector4 lmMean, int lmId, int poseIndex)
{
   // printf("Plane Factor: x%d -> l%d\n", poseIndex, lmId); fflush(stdout);
   graph.add(gtsam::OrientedPlane3Factor(lmMean, orientedPlaneNoise, gtsam::Symbol('x', poseIndex), gtsam::Symbol('l', lmId)));
}

void FactorGraphHandler::addGenericProjectionFactor(gtsam::Point2 point, int lmId, int poseIndex)
{
   graph.add(gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>(
         point, pointLandmarkNoise, gtsam::Symbol('x', poseIndex), gtsam::Symbol('p', lmId), K));
}

void FactorGraphHandler::setPoseInitialValue(int index, gtsam::Pose3 value)
{
   // printf("Pose Initial Value: x%d\n", index); fflush(stdout);
   if (structure.find('x' + std::to_string(index)) == structure.end())
   {
      structure.insert('x' + std::to_string(index));
      initial.insert(gtsam::Symbol('x', index), value);
   }
}


void FactorGraphHandler::setPointLandmarkInitialValue(int landmarkId, gtsam::Point3 value)
{
   if (!initial.exists(gtsam::Symbol('p', landmarkId)) && structure.find('p' + std::to_string(landmarkId)) == structure.end())
   {
      structure.insert('p' + std::to_string(landmarkId));
      initial.insert(gtsam::Symbol('p', landmarkId), value);
   }
}

void FactorGraphHandler::setOrientedPlaneInitialValue(int landmarkId, gtsam::OrientedPlane3 value)
{
   // printf("Plane Initial Value: l%d\n", landmarkId); fflush(stdout);
   if (!initial.exists(gtsam::Symbol('l', landmarkId)) && structure.find('l' + std::to_string(landmarkId)) == structure.end())
   {
      structure.insert('l' + std::to_string(landmarkId));
      initial.insert(gtsam::Symbol('l', landmarkId), value);
   }
}

void FactorGraphHandler::optimize()
{
   result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
}

void FactorGraphHandler::optimizeISAM2(uint8_t numberOfUpdates)
{
   isam.update(graph, initial);
   for (uint8_t i = 1; i < numberOfUpdates; i++)
   {
      isam.update();
   }
   result = isam.calculateEstimate();
}

void FactorGraphHandler::clearISAM2()
{
   initial.clear();
   graph.resize(0);
}

const gtsam::NonlinearFactorGraph& FactorGraphHandler::getFactorGraph()
{
   return graph;
}

void FactorGraphHandler::SLAMTest()
{
   using namespace gtsam;

   int currentPoseId = 1;

   Pose3 init_pose(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
   addPriorPoseFactor(currentPoseId, Pose3::Identity());
   setPoseInitialValue(currentPoseId, Pose3::Identity());

   addOrientedPlaneFactor(Vector4(1, 0, 0, -3), 0, currentPoseId);
   setOrientedPlaneInitialValue(0, gtsam::OrientedPlane3(Vector4(0.8, 0.1, 0.1, -2.9)));

   addOrientedPlaneFactor(Vector4(0, 0, 1, -3), 1, currentPoseId);
   setOrientedPlaneInitialValue(1, gtsam::OrientedPlane3(Vector4(0.1, 0.04, 1.1, -2.8)));

   Pose3 odometry(Rot3::Ypr(0.0, 0.0, 0.0), Point3(1.0, 0.0, 0.0));
   addOdometryFactor(odometry, 1);
   setPoseInitialValue(currentPoseId, odometry);

   addOrientedPlaneFactor(Vector4(1, 0, 0, -2), 0, currentPoseId);
   setOrientedPlaneInitialValue(0, gtsam::OrientedPlane3(Vector4(0.8, 0.1, 0.1, -2.1)));

   addOrientedPlaneFactor(Vector4(0, 0, 1, -3), 1, currentPoseId);
   setOrientedPlaneInitialValue(1, gtsam::OrientedPlane3(Vector4(0.1, 0.04, 1.1, -2.2)));

   optimize();

   result.print("Result Planes");
}

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

// We will also need a camera object to hold calibration information and perform projections.
#include <gtsam/geometry/SimpleCamera.h>

/* ************************************************************************* */
std::vector<gtsam::Point3> createPoints() {

   // Create the set of ground-truth landmarks
   std::vector<gtsam::Point3> points;
   points.push_back(gtsam::Point3(10.0,10.0,10.0));
   points.push_back(gtsam::Point3(-10.0,10.0,10.0));
   points.push_back(gtsam::Point3(-10.0,-10.0,10.0));
   points.push_back(gtsam::Point3(10.0,-10.0,10.0));
   points.push_back(gtsam::Point3(10.0,10.0,-10.0));
   points.push_back(gtsam::Point3(-10.0,10.0,-10.0));
   points.push_back(gtsam::Point3(-10.0,-10.0,-10.0));
   points.push_back(gtsam::Point3(10.0,-10.0,-10.0));

   return points;
}

/* ************************************************************************* */
std::vector<gtsam::Pose3> createPoses() {

   // Create the set of ground-truth poses
   std::vector<gtsam::Pose3> poses;
   double radius = 30.0;
   int i = 0;
   double theta = 0.0;
   gtsam::Point3 up(0,0,1);
   gtsam::Point3 target(0,0,0);
   for(; i < 8; ++i, theta += 2*M_PI/8) {

      printf("Angle (i): %.3lf\n", theta);

      gtsam::Point3 position = gtsam::Point3(radius*cos(theta), radius*sin(theta), 0.0);
      gtsam::SimpleCamera camera = gtsam::SimpleCamera::Lookat(position, target, up);

      printf("Position: %.3lf, %.3lf, %.3lf\n", position.x(), position.y(), position.z());

      gtsam::Pose3 camPose = camera.pose();

      poses.push_back(camera.pose());
   }
   return poses;
}
/* ************************************************************************* */


void FactorGraphHandler::VisualSLAMTest()
{
   using namespace gtsam;
   using namespace std;
   // Define the camera calibration parameters
   gtsam::Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));

   // Define the camera observation noise model, 1 pixel stddev
   auto measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0);

   // Create the set of ground-truth landmarks
   vector<Point3> points = createPoints();

   // Create the set of ground-truth poses
   vector<Pose3> poses = createPoses();

   // Create an iSAM2 object. Unlike iSAM1, which performs periodic batch steps
   // to maintain proper linearization and efficient variable ordering, iSAM2
   // performs partial relinearization/reordering at each step. A parameter
   // structure is available that allows the user to set various properties, such
   // as the relinearization threshold and type of linear solver. For this
   // example, we we set the relinearization threshold small so the iSAM2 result
   // will approach the batch result.
   ISAM2Params parameters;
   parameters.relinearizeThreshold = 0.01;
   parameters.relinearizeSkip = 1;
   ISAM2 isam(parameters);

   // Create a Factor Graph and Values to hold the new data
   NonlinearFactorGraph graph;
   Values initialEstimate;

   // Loop over the poses, adding the observations to iSAM incrementally
   for (size_t i = 0; i < poses.size(); ++i) {


      printf("Pose: (%ld) \n", i);

      // Add factors for each landmark observation
      for (size_t j = 0; j < points.size(); ++j) {
         PinholeCamera<Cal3_S2> camera(poses[i], *K);
         Point2 measurement = camera.project(points[j]);

         printf("Measurement (%ld): %.4lf, %.4lf\n", j, measurement.x(), measurement.y());

         graph.add(gtsam::GenericProjectionFactor<Pose3, Point3, Cal3_S2>(
               measurement, measurementNoise, Symbol('x', i), Symbol('l', j), K));
      }

      // Add an initial guess for the current pose
      // Intentionally initialize the variables off from the ground truth
      static Pose3 kDeltaPose(Rot3::Rodrigues(-0.1, 0.2, 0.25),
                              Point3(0.05, -0.10, 0.20));
      initialEstimate.insert(Symbol('x', i), poses[i] * kDeltaPose);

      // If this is the first iteration, add a prior on the first pose to set the
      // coordinate frame and a prior on the first landmark to set the scale Also,
      // as iSAM solves incrementally, we must wait until each is observed at
      // least twice before adding it to iSAM.
      if (i == 0) {
         // Add a prior on pose x0, 30cm std on x,y,z and 0.1 rad on roll,pitch,yaw
         static auto kPosePrior = noiseModel::Diagonal::Sigmas(
               (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3))
                     .finished());
         graph.addPrior(Symbol('x', 0), poses[0], kPosePrior);

         // Add a prior on landmark l0
         static auto kPointPrior = noiseModel::Isotropic::Sigma(3, 0.1);
         graph.addPrior(Symbol('l', 0), points[0], kPointPrior);

         // Add initial guesses to all observed landmarks
         // Intentionally initialize the variables off from the ground truth
         static Point3 kDeltaPoint(-0.25, 0.20, 0.15);
         for (size_t j = 0; j < points.size(); ++j)
            initialEstimate.insert<Point3>(Symbol('l', j), points[j] + kDeltaPoint);

      } else {

         graph.print("Graph");
         initialEstimate.print("Initial");
         isam.printStats();
         isam.print("ISAM");

         // Update iSAM with the new factors
         isam.update(graph, initialEstimate);
         // Each call to iSAM2 update(*) performs one iteration of the iterative
         // nonlinear solver. If accuracy is desired at the expense of time,
         // update(*) can be called additional times to perform multiple optimizer
         // iterations every step.
         isam.update();
         Values currentEstimate = isam.calculateEstimate();
         cout << "****************************************************" << endl;
         cout << "Frame " << i << ": " << endl;
         currentEstimate.print("Current estimate: ");

         // Clear the factor graph and values for the next iteration
         graph.resize(0);
         initialEstimate.clear();
      }
   }
}