import numpy as np
import os
import open3d as o3d

def icp_open3d(source, target, initial_transformation=np.eye(4), max_iterations=200):
    threshold = 0.05  # Soglia di distanza per ICP
    icp_result = o3d.pipelines.registration.registration_icp(
        source, target, threshold, initial_transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iterations)
    )
    return icp_result.transformation



def hierarchy_slam_icp(input_all_pointcloud):  # Hierarchical SLAM using ICP

    # Skip the first pointcloud, as it is only the target for the first alignment
    # For each subsequent pair of pointclouds, apply the transformation from the previous epoch to the current source pointcloud
    # The i-th transformation is reused from the last epoch to align the pointcloud
    # This process results in overlapping pointclouds being combined sequentially

    # Initial pointclouds for the first epoch
    epoch_start_pointclouds = input_all_pointcloud
    epoch = 0

    # Initialize the last epoch transformations with identity matrices
    # Identity matrix represents no transformation initially
    last_epoch_transformation = [np.eye(4) for _ in range(len(input_all_pointcloud))]

    while len(epoch_start_pointclouds) > 1:
        epoch += 1
        i = 1
        dict_analysis = {}
        new_halfed_pointcloud = []
        transformation_current = []

        print(f"START Epoch {epoch}, PCs: {len(epoch_start_pointclouds)}")

        while i < len(epoch_start_pointclouds):

            if len(epoch_start_pointclouds) == 2:
                print("LAST EPOCH")
                print(len(last_epoch_transformation))
                print(len(epoch_start_pointclouds))

            if i > 0:
                # Fetch transformations from the previous epoch for the current pair of pointclouds
                initial_transform_from_last_epoch_1_couple = last_epoch_transformation[i-1]
                initial_transform_from_last_epoch_2_couple = last_epoch_transformation[i]

                # Combine prior transformations to reduce cumulative error
                # This is done by matrix multiplication of transformations from previous epochs
                prior_transformation_composed = np.dot(initial_transform_from_last_epoch_1_couple, initial_transform_from_last_epoch_2_couple)

                # Transform the current source pointcloud using the composed prior transformation
                source_raw = epoch_start_pointclouds[i]
                current_source = o3d.geometry.PointCloud(source_raw)
                current_source.transform(prior_transformation_composed)

                # Target pointcloud is the previous one in the sequence
                target = epoch_start_pointclouds[i-1]

                # Apply ICP (Iterative Closest Point) algorithm to align the current source to the target
                updated_transform_icp_result = icp_open3d(current_source, target)

                # Total transformation is the combination of the prior transformation and the ICP result
                total_transformation_prior_plus_icp = np.dot(prior_transformation_composed, updated_transform_icp_result)

                # Apply the ICP result transformation to the source pointcloud
                transformed_icp_source = o3d.geometry.PointCloud(current_source)
                transformed_icp_source.transform(updated_transform_icp_result)

                # Merge the target and the transformed source pointclouds
                merged_pcd = target + transformed_icp_source
                transformation_current.append(total_transformation_prior_plus_icp)
                new_halfed_pointcloud.append(merged_pcd)

                # Handle the case where the number of pointclouds is odd
                if i == len(epoch_start_pointclouds) - 2 and len(epoch_start_pointclouds) % 2 != 0:
                    print("ODD NUMBER OF POINTCLOUDS")
                    print(f"Epoch {epoch}, i = {i}, PCs AVAILABLE: {len(epoch_start_pointclouds)} PCS COMP: {len(new_halfed_pointcloud)} TRANSFORM: {len(transformation_current)}")
                    print("Adding last PC with its transformation")
                    last_pc = epoch_start_pointclouds[i+1]
                    last_transform = last_epoch_transformation[i+1]

                    transformation_current.append(last_transform)
                    new_halfed_pointcloud.append(last_pc)

            i += 2

        # Prepare for the next epoch
        epoch_start_pointclouds = new_halfed_pointcloud
        last_epoch_transformation = transformation_current
        print(f"Computed Epoch {epoch}, PCs created: {len(new_halfed_pointcloud)}")

        dict_analysis[epoch] = [transformation_current, new_halfed_pointcloud]

    # Return the final merged pointcloud
    return new_halfed_pointcloud[0]