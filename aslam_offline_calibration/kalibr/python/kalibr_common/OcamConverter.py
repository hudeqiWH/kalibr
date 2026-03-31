#!/usr/bin/env python3
"""
Ocam Camera Parameter Converter for Kalibr

Converts Ocam parameters from JSON format (e.g., from park_front.json)
to Kalibr YAML configuration format.

Usage:
    python OcamConverter.py --input park_front.json --output ocam_camchain.yaml
"""

import json
import yaml
import argparse
import sys


def convert_ocam_json_to_kalibr(json_file, camera_name="cam0", rostopic="/camera/image_raw"):
    """
    Convert Ocam JSON parameters to Kalibr format.
    
    Args:
        json_file: Path to Ocam JSON file
        camera_name: Name of the camera in Kalibr config
        rostopic: ROS topic for the camera
    
    Returns:
        Dictionary in Kalibr configuration format
    """
    with open(json_file, 'r') as f:
        ocam_data = json.load(f)
    
    intrinsic = ocam_data['intrinsic_param']
    
    # Extract parameters
    cx = intrinsic['principal_point'][0]
    cy = intrinsic['principal_point'][1]
    c = intrinsic['affine_c']
    d = intrinsic['affine_d']
    e = intrinsic['affine_e']
    
    world2cam = intrinsic['cam2world']  # Note: naming is swapped in original format
    cam2world = intrinsic['world2cam']  # These are actually the inverse polynomials
    
    width = intrinsic['camera_width']
    height = intrinsic['camera_height']
    
    # Build intrinsics list
    intrinsics = [
        cx, cy,           # principal point
        c, d, e,          # affine parameters
        len(world2cam),   # world2cam polynomial length
        len(cam2world),   # cam2world polynomial length
    ]
    intrinsics.extend(world2cam)
    intrinsics.extend(cam2world)
    
    # Build Kalibr config
    kalibr_config = {
        camera_name: {
            'camera_model': 'ocam',
            'intrinsics': intrinsics,
            'distortion_model': 'none',
            'distortion_coeffs': [],
            'resolution': [width, height],
            'rostopic': rostopic
        }
    }
    
    return kalibr_config


def convert_multiple_cameras(json_files, camera_names=None, rostopics=None):
    """
    Convert multiple Ocam JSON files to a single Kalibr camera chain.
    
    Args:
        json_files: List of paths to Ocam JSON files
        camera_names: List of camera names (optional)
        rostopics: List of ROS topics (optional)
    
    Returns:
        Dictionary with all camera configurations
    """
    if camera_names is None:
        camera_names = [f"cam{i}" for i in range(len(json_files))]
    
    if rostopics is None:
        rostopics = [f"/camera{i}/image_raw" for i in range(len(json_files))]
    
    config = {}
    for json_file, name, topic in zip(json_files, camera_names, rostopics):
        camera_config = convert_ocam_json_to_kalibr(json_file, name, topic)
        config.update(camera_config)
    
    return config


def main():
    parser = argparse.ArgumentParser(
        description='Convert Ocam JSON parameters to Kalibr YAML format'
    )
    parser.add_argument('--input', '-i', nargs='+', required=True,
                       help='Input Ocam JSON file(s)')
    parser.add_argument('--output', '-o', required=True,
                       help='Output Kalibr YAML file')
    parser.add_argument('--names', '-n', nargs='+',
                       help='Camera names (default: cam0, cam1, ...)')
    parser.add_argument('--topics', '-t', nargs='+',
                       help='ROS topics (default: /camera0/image_raw, ...)')
    parser.add_argument('--target', type=str,
                       help='Target configuration YAML file to include')
    
    args = parser.parse_args()
    
    # Validate inputs
    if args.names and len(args.names) != len(args.input):
        print("Error: Number of camera names must match number of input files")
        sys.exit(1)
    
    if args.topics and len(args.topics) != len(args.input):
        print("Error: Number of topics must match number of input files")
        sys.exit(1)
    
    # Convert
    config = convert_multiple_cameras(
        args.input,
        args.names,
        args.topics
    )
    
    # Add target config if provided
    if args.target:
        with open(args.target, 'r') as f:
            target_config = yaml.safe_load(f)
        if 'target' in target_config:
            config['target'] = target_config['target']
    
    # Write output
    with open(args.output, 'w') as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)
    
    print(f"Successfully converted {len(args.input)} camera(s) to {args.output}")
    
    # Print summary
    for name in (args.names or [f"cam{i}" for i in range(len(args.input))]):
        cam_config = config[name]
        print(f"\n{name}:")
        print(f"  Model: {cam_config['camera_model']}")
        print(f"  Resolution: {cam_config['resolution']}")
        print(f"  Principal point: ({cam_config['intrinsics'][0]:.2f}, {cam_config['intrinsics'][1]:.2f})")
        print(f"  w2c polynomial degree: {cam_config['intrinsics'][5]}")
        print(f"  c2w polynomial degree: {cam_config['intrinsics'][6]}")


if __name__ == '__main__':
    main()
