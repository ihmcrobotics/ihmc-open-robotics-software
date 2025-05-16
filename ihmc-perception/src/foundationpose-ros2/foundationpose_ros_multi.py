import sys
import os

# sys.path.append(os.path.join(os.path.dirname(__file__), 'FoundationPose'))
base_dir = os.path.dirname(os.path.abspath(__file__))  # Just get the current script folder
sys.path.append(os.path.join(base_dir, 'FoundationPose'))
sys.path.append(os.path.join(base_dir, 'FoundationPose', 'nvdiffrast'))


import rclpy
from rclpy.node import Node
from estimater import *
import cv2
import numpy as np
import trimesh
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseStamped
from cv_bridge import CvBridge
import argparse
import os
from scipy.spatial.transform import Rotation as R
from ultralytics import SAM
from cam_2_base_transform import *
import os
import tkinter as tk
from tkinter import Listbox, END, Button
import glob

# Save the original `__init__` and `register` methods
original_init = FoundationPose.__init__
original_register = FoundationPose.register

# Modify `__init__` to add `is_register` attribute
def modified_init(self, model_pts, model_normals, symmetry_tfs=None, mesh=None, scorer=None, refiner=None, glctx=None, debug=0, debug_dir='./FoundationPose'):
    original_init(self, model_pts, model_normals, symmetry_tfs, mesh, scorer, refiner, glctx, debug, debug_dir)
    self.is_register = False  # Initialize as False

# Modify `register` to set `is_register` to True when a pose is registered
def modified_register(self, K, rgb, depth, ob_mask, iteration):
    pose = original_register(self, K, rgb, depth, ob_mask, iteration)
    self.is_register = True  # Set to True after registration
    return pose

# Apply the modifications
FoundationPose.__init__ = modified_init
FoundationPose.register = modified_register

class FileSelectorGUI:
    def __init__(self, master, file_paths):
        self.master = master
        self.master.title("Library: Sequence Selector")
        self.file_paths = file_paths
        self.reordered_paths = None  # Store the reordered paths here

        # Create a listbox to display the file names
        self.listbox = Listbox(master, selectmode="extended", width=50, height=10)
        self.listbox.pack()

        # Populate the listbox with file names without extensions
        for file_path in self.file_paths:
            file_name = os.path.splitext(os.path.basename(file_path))[0]
            self.listbox.insert(END, file_name)

        # Buttons for rearranging the order
        self.up_button = Button(master, text="Move Up", command=self.move_up)
        self.up_button.pack(side="left", padx=5, pady=5)

        self.down_button = Button(master, text="Move Down", command=self.move_down)
        self.down_button.pack(side="left", padx=5, pady=5)

        self.done_button = Button(master, text="Done", command=self.done)
        self.done_button.pack(side="left", padx=5, pady=5)

    def move_up(self):
        """Move selected items up in the listbox."""
        selected_indices = list(self.listbox.curselection())
        for index in selected_indices:
            if index > 0:
                # Swap with the previous item
                file_name = self.listbox.get(index)
                self.listbox.delete(index)
                self.listbox.insert(index - 1, file_name)
                self.listbox.selection_set(index - 1)

    def move_down(self):
        """Move selected items down in the listbox."""
        selected_indices = list(self.listbox.curselection())
        for index in reversed(selected_indices):
            if index < self.listbox.size() - 1:
                # Swap with the next item
                file_name = self.listbox.get(index)
                self.listbox.delete(index)
                self.listbox.insert(index + 1, file_name)
                self.listbox.selection_set(index + 1)

    def done(self):
        """Save the reordered paths and close the GUI."""
        reordered_file_names = self.listbox.get(0, END)

        # Recreate the full file paths based on the reordered file names (without extensions)
        file_name_to_full_path = {
            os.path.splitext(os.path.basename(file))[0]: file for file in self.file_paths
        }
        self.reordered_paths = [file_name_to_full_path[file_name] for file_name in reordered_file_names]

        # Print the reordered paths
        print("Reordered file paths:")
        for path in self.reordered_paths:
            print(path)

        # Close the GUI
        self.master.quit()

    def get_reordered_paths(self):
        """Return the reordered file paths after the GUI has closed."""
        return self.reordered_paths

# Example usage
def rearrange_files(file_paths):
    root = tk.Tk()
    app = FileSelectorGUI(root, file_paths)
    root.mainloop()  # Start the GUI event loop
    return app.get_reordered_paths()  # Return the reordered paths after GUI closes

# Argument Parser
parser = argparse.ArgumentParser()
code_dir = os.path.dirname(os.path.realpath(__file__))
parser.add_argument('--est_refine_iter', type=int, default=4)
parser.add_argument('--track_refine_iter', type=int, default=2)
args = parser.parse_args()

class PoseEstimationNode(Node):
    def __init__(self, new_file_paths):
        super().__init__('pose_estimation_node')
        
        # ROS subscriptions and publishers
        self.image_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)
        
        self.bridge = CvBridge()
        self.depth_image = None
        self.color_image = None
        self.cam_K = None  # Initialize cam_K as None until we receive the camera info
        
        # Load meshes
        self.mesh_files = new_file_paths
        self.meshes = [trimesh.load(mesh) for mesh in self.mesh_files]
        
        self.bounds = [trimesh.bounds.oriented_bounds(mesh) for mesh in self.meshes]
        self.bboxes = [np.stack([-extents/2, extents/2], axis=0).reshape(2, 3) for _, extents in self.bounds]
        
        self.scorer = ScorePredictor()
        self.refiner = PoseRefinePredictor()
        self.glctx = dr.RasterizeCudaContext()

        # Initialize SAM2 model
        self.seg_model = SAM("sam2.1_b.pt")

        self.pose_estimations = {}  # Dictionary to track multiple pose estimations
        self.pose_publishers = {}  # Dictionary to store publishers for each object
        self.tracked_objects = []  # Initialize to store selected objects' masks
        self.i = 0

    def camera_info_callback(self, msg):
        if self.cam_K is None:  # Update cam_K only once to avoid redundant updates
            self.cam_K = np.array(msg.k).reshape((3, 3))
            self.get_logger().info(f"Camera intrinsic matrix initialized: {self.cam_K}")

    def image_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1") / 1e3
        self.process_images()

    def process_images(self):
        if self.color_image is None or self.depth_image is None or self.cam_K is None:
            return

        H, W = self.color_image.shape[:2]
        color = cv2.resize(self.color_image, (W, H), interpolation=cv2.INTER_NEAREST)
        depth = cv2.resize(self.depth_image, (W, H), interpolation=cv2.INTER_NEAREST)
        depth[(depth < 0.1) | (depth >= np.inf)] = 0

        if self.i == 0:
            masks_accepted = False

            while not masks_accepted:
                # Use SAM2 for segmentation
                res = self.seg_model.predict(color)[0]
                res.save("masks.png")
                if not res:
                    self.get_logger().warn("No masks detected by SAM2.")
                    return

                objects_to_track = []

                # Iterate over the segmentation results to extract the masks and bounding boxes
                for r in res:
                    img = np.copy(r.orig_img)
                    for ci, c in enumerate(r):
                        mask = np.zeros((H, W), np.uint8)
                        contour = c.masks.xy.pop().astype(np.int32).reshape(-1, 1, 2)
                        _ = cv2.drawContours(mask, [contour], -1, (255, 255, 255), cv2.FILLED)

                        # Store mask and bounding box
                        objects_to_track.append({
                            'mask': mask,
                            'box': c.boxes.xyxy.tolist().pop(),
                            'contour': contour
                        })

                if not objects_to_track:
                    self.get_logger().warn("No objects found in the image.")
                    return

                self.tracked_objects = []  # Reset tracked objects for redo
                temporary_pose_estimations = {}
                skipped_indices = []  # Track skipped objects' indices

                def click_event(event, x, y, flags, params):
                    if event == cv2.EVENT_LBUTTONDOWN:
                        closest_dist = float('inf')
                        selected_obj = None

                        for obj in objects_to_track:
                            if obj['mask'][y, x] == 255:  # Check if click is inside the mask
                                dist = cv2.pointPolygonTest(obj['contour'], (x, y), True)

                                if dist < closest_dist:
                                    closest_dist = dist
                                    selected_obj = obj

                        if selected_obj is not None:
                            sequential_id = len(self.tracked_objects) + len(skipped_indices)
                            self.get_logger().info(f"Object {sequential_id} selected.")
                            self.tracked_objects.append(selected_obj['mask'])

                            # Temporarily store the mesh and bounds to avoid permanent removal
                            temp_mesh = self.meshes.pop(0)  # Remove the first mesh in line
                            temp_to_origin, _ = self.bounds.pop(0)  # Remove the first bound in line

                            # Initialize FoundationPose for each detected object with corresponding mesh
                            pose_est = FoundationPose(
                                model_pts=temp_mesh.vertices,
                                model_normals=temp_mesh.vertex_normals,
                                mesh=temp_mesh,
                                scorer=self.scorer,
                                refiner=self.refiner,
                                glctx=self.glctx
                            )

                            temporary_pose_estimations[sequential_id] = {
                                'pose_est': pose_est,
                                'mask': selected_obj['mask'],
                                'to_origin': temp_to_origin
                            }

                            # Refresh the dialog box with the updated object name
                            refresh_dialog_box()

                def refresh_dialog_box():
                    # Display contours for all detected objects
                    combined_mask_image = np.copy(color)
                    for idx, obj in enumerate(objects_to_track):
                        cv2.drawContours(combined_mask_image, [obj['contour']], -1, (0, 255, 0), 2)  # Green contours

                    # Get the next mesh name for user guidance, accounting for skips
                    next_mesh_idx = len(self.tracked_objects) + len(skipped_indices)
                    if next_mesh_idx < len(self.mesh_files):
                        next_mesh_name = os.path.basename(self.mesh_files[next_mesh_idx].split("/")[-1].split(".")[0])
                    else:
                        next_mesh_name = "None"

                    # Create the dialog box overlay
                    overlay = combined_mask_image.copy()
                    dialog_text = (
                        f"Next object to select: {next_mesh_name}\n"
                        "Instructions:\n"
                        "- Click on the object to select.\n"
                        "- Press 's' to skip the current object.\n"
                        "- Press 'c', 'Enter', or 'Space' to confirm selection.\n"
                        "- Press 'r' to redo mask selection.\n"
                        "- Press 'q' to quit.\n"
                    )
                    y0, dy = 30, 20
                    for i, line in enumerate(dialog_text.split('\n')):
                        y = y0 + i * dy
                        cv2.putText(overlay, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

                    cv2.imshow('Click on objects to track', overlay)
                    cv2.setMouseCallback('Click on objects to track', click_event)

                refresh_dialog_box()

                while True:
                    key = cv2.waitKey(0)  # Wait for a key event
                    if key == ord('r'):
                        self.get_logger().info("Redoing mask selection.")
                        break  # Break the inner loop to redo mask selection
                    elif key == ord('s'):
                        self.get_logger().info("Skipping current object.")
                        skipped_indices.append(len(self.tracked_objects) + len(skipped_indices))  # Track skipped mesh index

                        # Remove the first mesh and bounds in line
                        self.meshes.pop(0)
                        self.bounds.pop(0)

                        refresh_dialog_box()
                    elif key in [ord('q'), 27]:  # 'q' or Esc to quit
                        self.get_logger().info("Quitting mask selection.")
                        return
                    elif key in [ord('c'), 13, 32]:  # 'c', Enter, or Space to confirm
                        if self.tracked_objects:
                            # Confirm the selection and update the actual pose_estimations
                            self.pose_estimations = temporary_pose_estimations

                            # Remove the corresponding meshes and bounds from the original lists only after confirmation
                            selected_indices = sorted(temporary_pose_estimations.keys(), reverse=True)
                            self.meshes = [self.meshes[idx] for idx in selected_indices]
                            self.bounds = [self.bounds[idx] for idx in selected_indices]

                            masks_accepted = True  # Exit the outer loop if masks are accepted
                            break
                        else:
                            self.get_logger().warn("No objects selected. Redo mask selection.")

        visualization_image = np.copy(color)

        for idx, data in self.pose_estimations.items():
            pose_est = data['pose_est']
            obj_mask = data['mask']
            to_origin = data['to_origin']
            if pose_est.is_register:
                pose = pose_est.track_one(rgb=color, depth=depth, K=self.cam_K, iteration=args.track_refine_iter)
                center_pose = pose @ np.linalg.inv(to_origin)

                self.publish_pose_stamped(center_pose, f"object_{idx}_frame", f"/Current_OBJ_position_{idx+1}")

                visualization_image = self.visualize_pose(visualization_image, center_pose, idx)
            else:
                pose = pose_est.register(K=self.cam_K, rgb=color, depth=depth, ob_mask=obj_mask, iteration=args.est_refine_iter)
            self.i += 1

        cv2.imshow('Pose Estimation & Tracking', visualization_image[..., ::-1])
        cv2.waitKey(1)

    def visualize_pose(self, image, center_pose, idx):
        bbox = self.bboxes[idx % len(self.bboxes)]
        vis = draw_posed_3d_box(self.cam_K, img=image, ob_in_cam=center_pose, bbox=bbox)
        vis = draw_xyz_axis(vis, ob_in_cam=center_pose, scale=0.1, K=self.cam_K, thickness=3, transparency=0, is_input_rgb=True)
        return vis

    def publish_pose_stamped(self, center_pose, frame_id, topic_name):
        if topic_name not in self.pose_publishers:
            self.pose_publishers[topic_name] = self.create_publisher(PoseStamped, topic_name, 10)
        
        # Convert the center_pose matrix to a PoseStamped message
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        pose_stamped_msg.header.frame_id = frame_id

        # Convert center_pose to the pose format
        position = center_pose[:3, 3]
        rotation_matrix = center_pose[:3, :3]
        quaternion = R.from_matrix(rotation_matrix).as_quat()

        # Combine position and quaternion into a single array
        pose_array = np.concatenate((position, quaternion))

        # Apply transformation to convert from camera to base frame
        transformed_pose = transformation(pose_array)

        # Populate PoseStamped message with transformed pose
        pose_stamped_msg.pose.position.x = transformed_pose[0]
        pose_stamped_msg.pose.position.y = transformed_pose[1]
        pose_stamped_msg.pose.position.z = transformed_pose[2]

        pose_stamped_msg.pose.orientation.w = transformed_pose[3]
        pose_stamped_msg.pose.orientation.x = transformed_pose[4]
        pose_stamped_msg.pose.orientation.y = transformed_pose[5]
        pose_stamped_msg.pose.orientation.z = transformed_pose[6]

        # Publish the transformed pose
        self.pose_publishers[topic_name].publish(pose_stamped_msg)

def main(args=None):
    # Get the base directory (absolute path) where the script is located
    script_dir = os.path.dirname(os.path.realpath(__file__))
    
    # Join the base directory with the relative path to 'demo_data'
    source_directory = os.path.join(script_dir, "demo_data")
    file_paths = glob.glob(os.path.join(source_directory, '**', '*.obj'), recursive=True) + \
                 glob.glob(os.path.join(source_directory, '**', '*.stl'), recursive=True) + \
                 glob.glob(os.path.join(source_directory, '**', '*.STL'), recursive=True)

    # Call the function to rearrange files through the GUI
    new_file_paths = rearrange_files(file_paths)

    rclpy.init(args=args)
    node = PoseEstimationNode(new_file_paths)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
