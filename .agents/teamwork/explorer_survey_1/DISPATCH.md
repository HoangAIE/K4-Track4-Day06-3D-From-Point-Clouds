# Task Assignment: Survey Codebase, Starter Projection, & Datasets
- Read d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/ORIGINAL_REQUEST.md
- Investigate d:/K4-Track4-Day06-3D-From-Point-Clouds/starter/projection.py and any other starter files
- Inspect datasets: data/synthetic, data/kitti_mini, data/nuscenes_mini_subset (point cloud files, calibration files, images, labels/annotations)
- Analyze projection math requirements: velo_to_cam, cam_to_image, Tr_velo_to_cam, R0_rect, P2, homogenous coordinates, filtering, FoV
- Produce report at .agents/teamwork/explorer_survey_1/handoff.md

## 2026-10-07T09:30:25Z
You are the Codebase and Geometry Explorer for the Topic C LiDAR-Camera project.
Your working directory: d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/explorer_survey_1
Your task assignment: d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/explorer_survey_1/DISPATCH.md
MANDATORY: First read d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/ORIGINAL_REQUEST.md.

Investigate:
1. starter/projection.py: read existing code, examine TODO comments for `velo_to_cam` and `cam_to_image`.
2. Inspect data/synthetic, data/kitti_mini, data/nuscenes_mini_subset: examine directory structure, calibration format (calib.txt or JSON or matrix format), point cloud format (bin, pcd, npy, etc.), image format (png/jpg), label format (kitti 3D bounding boxes, annotations).
3. Test running `python -m starter.projection --help` or test running starter.projection with current code using .venv/Scripts/python.exe to see current behavior and errors.
4. Detail the exact mathematical formulas required for velo_to_cam, cam_to_image, coordinate transforms, filtering invalid / negative z / outside FOV points, and bounding box filtering.

Write your findings to d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/explorer_survey_1/handoff.md.
Also maintain progress.md in your working directory.
When finished, send a brief message to your parent with your completion status.
